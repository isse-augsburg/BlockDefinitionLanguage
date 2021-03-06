from setuptools import setup

package_name = 'swarm_sync'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='TODO',
    author_email='TODO@TODO.TODO',
    maintainer='TODO',
    maintainer_email='TODO@TODO.TODO',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_sync_node = swarm_sync.swarm_sync_node:main',
            'swarm_sync_client = swarm_sync.swarm_sync_client:main',
            'barrier_node = swarm_sync.barrier_node:main',

        ],
    },
)
