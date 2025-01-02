from setuptools import find_packages, setup

package_name = 'pwm_controller_generic'

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
    maintainer='rosuser',
    maintainer_email='BruceRayWilson42@gmail.com',
    description='Generic PWM controller for robotics applications',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
