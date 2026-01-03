from setuptools import find_packages, setup

package_name = 'hand_recognition'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hasan Çoban',
    maintainer_email='hasan@example.com',
    description='Hand gesture recognition node for robot control using MediaPipe',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_drive_node = hand_recognition.hand_drive_node:main',
        ],
    },
)

