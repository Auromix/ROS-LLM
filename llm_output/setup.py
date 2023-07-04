from setuptools import setup

package_name = 'llm_output'

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
    maintainer='hermanye',
    maintainer_email='hermanye233@icloud.com',
    description='The llm_input package contains input nodes for the ROS-LLM.',
    license="Apache-2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "llm_audio_output = llm_output.llm_audio_output:main",
        ],
    },
)
