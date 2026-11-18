from setuptools import setup

package_name = 'slalom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Slalomkings',
    maintainer_email='team13@ucsd.edu',
    description='Slalom nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Neuer Node = Datei in slalom/ anlegen + hier eine Zeile, z.B.:
            # 'cone_detector_opencv = slalom.cone_detector_opencv:main',
        ],
    },
)
