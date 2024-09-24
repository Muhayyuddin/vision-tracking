import os
from glob import glob
from setuptools import setup

package_name = 'nav_ontologies'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['launch/', 'owl_knowledge/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhayy',
    maintainer_email='muhayyuddin_1@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'owl_reasoner = nav_ontologies.owl_reasoner:main',
            'service = nav_ontologies.owl_knowledge_server:main',
            'client = nav_ontologies.owl_knowledge_client:main',

        ],
    },
)

