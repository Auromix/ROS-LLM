from setuptools import setup

package_name = 'llm_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'llm_config'],
    zip_safe=True,
    maintainer='hermanye',
    maintainer_email='hermanye233@icloud.com',
    description='The llm_robot package provides a ChatGPT function call server to simulate function calls for any robot',
    license="Apache-2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_robot = llm_robot.turtle_robot:main",
            "arm_robot = llm_robot.arx5_arm_robot:main",
            "multi_robot = llm_robot.multi_robot:main",
        ],
    },
)
