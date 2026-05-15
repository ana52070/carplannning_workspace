from setuptools import setup

package_name = 'myslam_car_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='2686367411@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_node = myslam_car_driver.yahboom_keyboard:main',
            'driver_node = myslam_car_driver.carplanning_driver:main',
            'robot_static_tf_pub_node = myslam_car_driver.robot_static_tf_pub:main'
        ],
    },
)
