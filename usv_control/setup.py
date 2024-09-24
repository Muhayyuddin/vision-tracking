from setuptools import setup

package_name = 'usv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhayy',
    maintainer_email='muhayyuddin_1@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pid_controlle = usv_control.usv_pid_control:main',
                'yaw_control = usv_control.yaw_controller:main',
                'position_control = usv_control.position_control:main',
                'los_control = usv_control.los_pid_controller:main',
                'twist_publisher = usv_control.twist_publisher:main',
                'smc_controller = usv_control.smc:main',
                'lqr_controller = usv_control.lqr:main',
                'pid_controller = usv_control.pid:main',



        ],
    },
)
