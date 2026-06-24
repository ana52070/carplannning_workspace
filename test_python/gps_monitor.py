#!/usr/bin/env python3
"""
GPS RTK Quality Monitor v2
=========================
准确监控 UM982 RTK 定位质量，修复 v1 的核心缺陷：
  v1 bug: status.status 来自 $GNHPR 的航向质量，不代表定位质量
  v2 fix: 用 position_covariance (来自 PVTSLNA bestpos_*std) 判断真实定位精度

功能：
  1. 基于协方差的精度分级（RTK Fixed / Float / DGPS / SPP）
  2. 区分显示「定位精度」和「航向质量」
  3. 静止漂移分析（距均值点的距离统计）
  4. CSV 日志记录（坐标、精度、状态、漂移量）
  5. 终端彩色实时输出

用法：
  python3 gps_monitor_v2.py [--rate 2.0] [--csv gps_log.csv] [--no-csv]
"""

import os
import sys
import math
import time
import argparse
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


# ============================================================
#  精度分级阈值（基于水平标准差 = sqrt(Cov_H)）
#  Cov_H = (cov[0] + cov[4]) / 2 = (latstd² + lonstd²) / 2
# ============================================================
# UM982 驱动里: cov[0] = bestpos_latstd², cov[4] = bestpos_lonstd²
# 所以 Cov_H 的 sqrt 就是平均水平标准差（米）
#
# 阈值参考：
#   RTK Fixed:  std < 0.03m  → Cov_H < 0.0009 m²
#   RTK Float:  std < 0.5m   → Cov_H < 0.25 m²
#   DGPS:       std < 2.0m   → Cov_H < 4.0 m²
#   SPP:        std >= 2.0m  → Cov_H >= 4.0 m²

GRADE_THRESHOLDS = [
    # (max_cov_h, label, color_code, emoji)
    (0.001,  "RTK_FIXED", "\033[92m", "🟢"),   # 绿色  std < ~3cm
    (0.01,   "RTK_GOOD",  "\033[96m", "🔵"),   # 青色  std < ~10cm
    (0.25,   "RTK_FLOAT", "\033[93m", "🟡"),   # 黄色  std < ~50cm
    (4.0,    "DGPS",      "\033[33m", "🟠"),   # 橙色  std < ~2m
    (float('inf'), "SPP", "\033[91m", "🔴"),   # 红色  std >= 2m
]

# NavSatFix status.status 的真实含义（在 UM982 驱动中）
# ⚠️ 注意：这里的 GBAS_FIX 实际代表 $GNHPR quality==4 (航向固定解)
#          与定位精度无关！
DRIVER_STATUS_TRUTH = {
    -1: ("NO_FIX",   "无定位"),
     0: ("FIX",      "航向:非固定解"),   # quality != 0,4,9
     1: ("SBAS_FIX", "航向:quality=9"),
     2: ("GBAS_FIX", "航向:固定解"),     # quality==4 (双天线航向RTK固定)
}

C_RESET = "\033[0m"
C_BOLD  = "\033[1m"
C_DIM   = "\033[2m"
C_WARN  = "\033[93m"
C_ERR   = "\033[91m"
C_OK    = "\033[92m"
C_CYAN  = "\033[96m"
C_MAG   = "\033[95m"


def grade_from_cov(cov_h: float):
    """根据 Cov_H 返回 (label, color, emoji)"""
    for max_cov, label, color, emoji in GRADE_THRESHOLDS:
        if cov_h < max_cov:
            return label, color, emoji
    return "SPP", "\033[91m", "🔴"


def haversine_m(lat1, lon1, lat2, lon2):
    """两个经纬度点之间的距离（米）"""
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class GpsMonitorV2(Node):
    def __init__(self, csv_path=None, report_interval=2.0):
        super().__init__('gps_monitor_v2')
        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self._cb, 10)

        self.report_interval = report_interval
        self.csv_path = csv_path
        self.csv_file = None
        self.csv_writer = None

        # --- 统计 ---
        self.msg_count = 0
        self.grade_counts = {}  # grade_label → count
        self.last_grade = None
        self.streak_count = 0
        self.streak_grade = None

        # --- 漂移分析 (滑动窗口) ---
        self.window_size = 200  # 最近 N 个点
        self.lat_window = deque(maxlen=self.window_size)
        self.lon_window = deque(maxlen=self.window_size)
        self.drift_max = 0.0
        self.drift_samples = deque(maxlen=self.window_size)

        # --- 协方差趋势 ---
        self.cov_window = deque(maxlen=60)  # 最近60个

        # --- 定时报告 ---
        self.last_report_time = time.time()
        self.last_change_msg = ""

        # --- CSV ---
        if csv_path:
            self._init_csv(csv_path)

        self.get_logger().info(f'GPS Monitor v2 started | report every {report_interval}s')
        if csv_path:
            self.get_logger().info(f'CSV logging to: {csv_path}')

    def _init_csv(self, path):
        self.csv_file = open(path, 'w')
        header = (
            "timestamp,ros_sec,msg_num,"
            "lat,lon,alt,"
            "cov_h,std_h,grade,"
            "nav_status,nav_status_text,"
            "drift_from_mean_m,"
            "lat_std,lon_std,alt_std\n"
        )
        self.csv_file.write(header)

    def _cb(self, msg: NavSatFix):
        self.msg_count += 1
        now = time.time()

        # --- 基础数据提取 ---
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        cov = msg.position_covariance
        cov_type = msg.position_covariance_type
        nav_status = msg.status.status

        # UM982 驱动: cov[0]=latstd², cov[4]=lonstd², cov[8]=hgtstd²
        lat_std = math.sqrt(abs(cov[0])) if cov[0] > 0 else 0.0
        lon_std = math.sqrt(abs(cov[4])) if cov[4] > 0 else 0.0
        alt_std = math.sqrt(abs(cov[8])) if cov[8] > 0 else 0.0
        cov_h = (cov[0] + cov[4]) / 2.0

        # --- 精度分级 ---
        grade, color, emoji = grade_from_cov(cov_h)
        std_h = math.sqrt(cov_h) if cov_h > 0 else 0.0

        # 统计
        self.grade_counts[grade] = self.grade_counts.get(grade, 0) + 1
        self.cov_window.append(cov_h)

        # --- 漂移分析 ---
        drift_m = 0.0
        if len(self.lat_window) >= 10:
            mean_lat = sum(self.lat_window) / len(self.lat_window)
            mean_lon = sum(self.lon_window) / len(self.lon_window)
            drift_m = haversine_m(mean_lat, mean_lon, lat, lon)
            self.drift_samples.append(drift_m)
            if drift_m > self.drift_max:
                self.drift_max = drift_m

        self.lat_window.append(lat)
        self.lon_window.append(lon)

        # --- 段切换检测 ---
        if grade != self.streak_grade:
            if self.streak_grade is not None and self.streak_count > 1:
                self.last_change_msg = (
                    f"  ── 上一段 [{self.streak_grade}] "
                    f"持续 {self.streak_count} 条 ──"
                )
            self.streak_grade = grade
            self.streak_count = 1
        else:
            self.streak_count += 1

        # --- 状态变化时立即输出详情 ---
        if grade != self.last_grade:
            if self.last_change_msg:
                print(f"{C_DIM}{self.last_change_msg}{C_RESET}")
                self.last_change_msg = ""

            status_name, status_desc = DRIVER_STATUS_TRUTH.get(
                nav_status, (f"?({nav_status})", "未知"))

            print(f"\n{'=' * 70}")
            print(f"  {emoji} [{color}{C_BOLD}{grade}{C_RESET}]  "
                  f"#{self.msg_count}  "
                  f"水平精度 std={std_h:.4f}m  Cov_H={cov_h:.6f}m²")
            print(f"  坐标: {lat:.10f}, {lon:.10f}, {alt:.2f}m")
            print(f"  分项: lat_std={lat_std:.4f}m  lon_std={lon_std:.4f}m  "
                  f"alt_std={alt_std:.4f}m")
            print(f"  驱动status: {status_name} ({status_desc})"
                  f"  {C_DIM}← ⚠️ 这是航向质量, 非定位质量!{C_RESET}")
            if drift_m > 0:
                print(f"  漂移: 距均值 {drift_m:.3f}m  "
                      f"历史最大 {self.drift_max:.3f}m")
            print(f"{'=' * 70}")

            self.last_grade = grade

        # --- 定时报告 ---
        if now - self.last_report_time >= self.report_interval:
            self._print_summary()
            self.last_report_time = now

        # --- CSV ---
        if self.csv_file:
            ros_sec = (msg.header.stamp.sec +
                       msg.header.stamp.nanosec * 1e-9)
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            status_name, _ = DRIVER_STATUS_TRUTH.get(
                nav_status, (f"?({nav_status})", ""))
            self.csv_file.write(
                f"{ts},{ros_sec:.3f},{self.msg_count},"
                f"{lat:.10f},{lon:.10f},{alt:.4f},"
                f"{cov_h:.6f},{std_h:.4f},{grade},"
                f"{nav_status},{status_name},"
                f"{drift_m:.4f},"
                f"{lat_std:.4f},{lon_std:.4f},{alt_std:.4f}\n"
            )
            # 每 100 条刷一次
            if self.msg_count % 100 == 0:
                self.csv_file.flush()

    def _print_summary(self):
        total = self.msg_count
        if total == 0:
            return

        # 精度分级统计条
        parts = []
        for max_cov, label, color, emoji in GRADE_THRESHOLDS:
            cnt = self.grade_counts.get(label, 0)
            if cnt > 0:
                pct = cnt / total * 100
                parts.append(f"{emoji}{color}{label}:{pct:.1f}%{C_RESET}")

        # 协方差趋势
        if self.cov_window:
            cov_now = self.cov_window[-1]
            cov_avg = sum(self.cov_window) / len(self.cov_window)
            cov_min = min(self.cov_window)
            cov_max = max(self.cov_window)
            cov_str = (f"Cov_H: now={cov_now:.4f} "
                       f"avg={cov_avg:.4f} "
                       f"min={cov_min:.4f} max={cov_max:.4f}")
        else:
            cov_str = ""

        # 漂移统计
        drift_str = ""
        if self.drift_samples:
            d_avg = sum(self.drift_samples) / len(self.drift_samples)
            d_sorted = sorted(self.drift_samples)
            d_95 = d_sorted[int(len(d_sorted) * 0.95)] if len(d_sorted) > 20 else d_sorted[-1]
            drift_str = (f"  漂移: avg={d_avg:.3f}m  "
                         f"95%={d_95:.3f}m  max={self.drift_max:.3f}m")

        # 可用性判定
        fixed_cnt = self.grade_counts.get("RTK_FIXED", 0)
        good_cnt = self.grade_counts.get("RTK_GOOD", 0)
        nav_ready_pct = (fixed_cnt + good_cnt) / total * 100

        if nav_ready_pct >= 90:
            verdict = f"{C_OK}✅ 可导航 (RTK稳定){C_RESET}"
        elif nav_ready_pct >= 50:
            verdict = f"{C_WARN}⚠️  RTK不稳定 (导航风险高){C_RESET}"
        else:
            verdict = f"{C_ERR}❌ 不可导航 (定位精度不足){C_RESET}"

        print(f"\n  📊 #{total}  {' | '.join(parts)}")
        print(f"  {cov_str}")
        if drift_str:
            print(drift_str)
        print(f"  导航就绪: {verdict}")

    def destroy_node(self):
        # 最终统计
        total = self.msg_count
        if total > 0:
            print(f"\n{'=' * 70}")
            print(f"  {C_BOLD}最终统计 ({total} 条消息){C_RESET}")
            print(f"{'=' * 70}")
            for max_cov, label, color, emoji in GRADE_THRESHOLDS:
                cnt = self.grade_counts.get(label, 0)
                if cnt > 0:
                    pct = cnt / total * 100
                    bar_len = int(40 * pct / 100)
                    bar = "█" * bar_len + "░" * (40 - bar_len)
                    print(f"  {emoji} {color}{label:12s}{C_RESET} "
                          f"{cnt:6d} ({pct:5.1f}%) [{bar}]")

            if self.drift_samples:
                d_sorted = sorted(self.drift_samples)
                d_avg = sum(d_sorted) / len(d_sorted)
                d_95 = d_sorted[int(len(d_sorted) * 0.95)] if len(d_sorted) > 20 else d_sorted[-1]
                print(f"\n  漂移统计 (最近{len(d_sorted)}点):")
                print(f"    平均: {d_avg:.3f}m | "
                      f"95th: {d_95:.3f}m | "
                      f"最大: {self.drift_max:.3f}m")

            fixed_cnt = self.grade_counts.get("RTK_FIXED", 0)
            good_cnt = self.grade_counts.get("RTK_GOOD", 0)
            nav_pct = (fixed_cnt + good_cnt) / total * 100
            print(f"\n  导航可用率 (RTK_FIXED + RTK_GOOD): {nav_pct:.1f}%")

            if nav_pct >= 90:
                print(f"  {C_OK}结论: ✅ RTK 定位质量满足导航要求{C_RESET}")
            elif nav_pct >= 50:
                print(f"  {C_WARN}结论: ⚠️  RTK 不稳定, 频繁降级, 不建议导航{C_RESET}")
            else:
                print(f"  {C_ERR}结论: ❌ 定位精度严重不足, 无法导航{C_RESET}")
                print(f"  {C_DIM}  排查方向: NTRIP 差分源连接 → 基站距离 → "
                      f"天线安装遮挡 → UM982 配置{C_RESET}")

            if self.cov_window:
                avg_cov = sum(self.cov_window) / len(self.cov_window)
                avg_std = math.sqrt(avg_cov)
                print(f"\n  近期水平精度: 平均 std ≈ {avg_std:.3f}m "
                      f"(Cov_H ≈ {avg_cov:.4f}m²)")
                if avg_std > 0.5:
                    print(f"  {C_ERR}  ↳ 真正的 RTK Fixed 应该 < 0.03m, "
                          f"你的值差了 {avg_std/0.03:.0f} 倍{C_RESET}")

            print(f"{'=' * 70}")

        if self.csv_file:
            self.csv_file.close()
            print(f"\n  CSV 已保存: {self.csv_path}")

        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='GPS RTK Quality Monitor v2')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='报告间隔秒数 (default: 2.0)')
    parser.add_argument('--csv', type=str, default=None,
                        help='CSV 日志路径 (default: auto-generate)')
    parser.add_argument('--no-csv', action='store_true',
                        help='禁用 CSV 记录')
    args = parser.parse_args()

    # 默认 CSV 路径
    csv_path = args.csv
    if not args.no_csv and csv_path is None:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = f'gps_log_{ts}.csv'

    if args.no_csv:
        csv_path = None

    rclpy.init()
    node = GpsMonitorV2(csv_path=csv_path, report_interval=args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
