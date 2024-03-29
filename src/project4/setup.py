from setuptools import setup
import glob

package_name = 'project4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cwduffy01',
    maintainer_email='cwduffy01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = project4.simulation:main',
            'vel = project4.velocity_translator:main',
            'nav = project4.navigation_controller:main'
        ],
    },
)
