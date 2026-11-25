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
            'sim = slalom.sim:main',
            'driver = slalom.driver:main',
            'save_images = slalom.save_images:main',
            'detector = slalom.detector:main',
            'fusion = slalom.fusion:main',
            'ai_detector = slalom.ai_detector:main',
            'oak_ai = slalom.oak_ai:main',
        ],
    },
)
