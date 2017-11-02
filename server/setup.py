from setuptools import setup

package_name = 'xenogui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=['test'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Loïc Dauphin',
    author_email='astralien3000@yahoo.fr',
    maintainer='Loïc Dauphin',
    maintainer_email='astralien3000@yahoo.fr',
    keywords=['ROS', 'GUI', 'Xenomorphales'],
    classifiers=[
        'Intended Audience :: Robot Makers',
        'License :: GPL',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='GUI of Xenomorphales team',
    license='GPL',
    entry_points={
        'console_scripts': [
            'test = test:main',
        ],
    },
)
