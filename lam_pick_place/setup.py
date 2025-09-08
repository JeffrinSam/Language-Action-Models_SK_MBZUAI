from setuptools import setup

package_name = 'lam_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pick_place_rrt.launch.py',
            'launch/pick_place_full.launch.py',
            'launch/pick_place_full_run.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RRT planner, controller, recorder, and linear Gaussian dynamics learner for pick-and-place in Isaac Sim.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_planner_node = lam_pick_place.rrt_planner_node:main',
            'trajectory_controller_node = lam_pick_place.trajectory_controller_node:main',
            'pick_place_manager_node = lam_pick_place.pick_place_manager_node:main',
            'trajectory_recorder_node = lam_pick_place.trajectory_recorder_node:main',
            'linear_gaussian_learner = lam_pick_place.linear_gaussian_learner:main',
        ],
    },
)
