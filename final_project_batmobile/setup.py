from setuptools import setup

package_name = 'final_project_batmobile'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/final_project.launch.py']),
	('share/' + package_name + '/launch', ['launch/final_project_color.launch.py']),
	('share/' + package_name + '/launch', ['launch/final_project_lidar.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'cmd_arbiter = final_project_batmobile.cmd_arbiter:main',
		'constant_speed = final_project_batmobile.constant_speed:main',
		'stopsign_executable = final_project_batmobile.stopsign_detection_node:main',
		'color_detection_test = final_project_batmobile.color_detection_test:main',
		'cmd_arbiter_color = final_project_batmobile.cmd_arbiter_color:main',
		'color_detector = final_project_batmobile.color_detection_final:main',
		'lidar_test = final_project_batmobile.lidar_test:main',
		'lidar_detection_final = final_project_batmobile.lidar_detection_final:main',
		'cmd_arbiter_lidar = final_project_batmobile.cmd_arbiter_lidar:main',
		'lidar_test_2 = final_project_batmobile.lidar_test_2:main',
        ],
    },
)
