#!/usr/bin/env python

from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='cnspy_relative_pose_evaluation',
    version="0.0.0",
    author='Roland Jung',
    author_email='roland.jung@aau.at',
    description='Evaluation of relative pose measurements.',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/aau-cns/cnspy_relative_pose_evaluation/',
    project_urls={
        "Bug Tracker": "https://github.com/aau-cns/cnspy_relative_pose_evaluation/issues",
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],

    packages=find_packages(exclude=["test_*", "TODO*"]),
    python_requires='>=3.6',
    install_requires=['numpy', 'pandas', 'spatialmath-python', 'scipy', 'matplotlib', 'joblib', 'configparser', 'cnspy_numpy_utils', 'cnspy_timestamp_association', 'cnspy_trajectory_evaluation', 'cnspy_ranging_evaluation' ],
    entry_points={
        'console_scripts': [
            'RelPoseMeasEvaluationTool = cnspy_relative_pose_evaluation.RelPoseMeasEvaluationTool:main',
            'RelPoseMeasEvaluation = cnspy_relative_pose_evaluation.RelPoseMeasEvaluation:main',
            'RelPose_ROSBag2CSV = cnspy_relative_pose_evaluation.RelPose_ROSBag2CSV:main',
            'ROSBag_TrueRelPoses = cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses:main',
            'ROSBag_Poses2RelPose = cnspy_relative_pose_evaluation.ROSBag_Poses2RelPose:main',
        ],
    },
)



