from setuptools import setup

package_name = 'cart_pole_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Cart pole controller nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cart_pole_controller = cart_pole_commander.cart_pole_controller:main',
            'cart_pole_logger = cart_pole_commander.cart_pole_logger:main',
            'cart_pole_csv = cart_pole_commander.cart_pole_csv:main',
        ],
    },
)
