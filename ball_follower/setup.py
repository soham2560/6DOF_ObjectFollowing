from setuptools import find_packages, setup

package_name = 'ball_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='container_user',
    maintainer_email='sohampatil45939@gmail.com',
    description='UR5 robot arm trajectory follower',  # Updated description
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_follower = ball_follower.ball_follower:main'
        ],
    },
    python_requires='>=3.8',
)