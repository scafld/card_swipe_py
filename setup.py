from setuptools import setup

package_name = 'card_swipe_py'

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
    maintainer='erkoiv',
    maintainer_email='koiverik@gmail.com',
    description='Sends a test trigger to "trigger" topic',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'triggerer = card_swipe_py.trigger_publisher:main',
            'swiper = card_swipe_py.swiper_trigger_sub:main'
        ],
    },
)
