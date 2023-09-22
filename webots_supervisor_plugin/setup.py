from setuptools import find_packages, setup

package_name = 'webots_supervisor_plugin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeuz',
    maintainer_email='igallesio@mir-robots.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_supervisor_plugin = webots_supervisor_plugin.webots_supervisor_plugin:main'
        ],
    },
)
