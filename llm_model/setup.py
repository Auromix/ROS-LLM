from setuptools import setup

package_name = "llm_model"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "llm_config", "llm_interfaces"],
    zip_safe=True,
    maintainer="hermanye",
    maintainer_email="hermanye233@icloud.com",
    description="The llm_model package provides a conversational interface using the OpenAI API through the implementation of the ChatGPT node.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chatgpt = llm_model.chatgpt:main",
        ],
    },
)
