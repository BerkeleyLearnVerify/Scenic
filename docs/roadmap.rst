Project Roadmap
===============

This document describes the core areas of development for the Scenic project, as decided by the Steering Committee (see `governance`).

Our long-term vision is that Scenic becomes a foundational, widely-used, open-source representation and toolkit supporting the entire design lifecycle of autonomous intelligent cyber-physical systems (AI-CPS). Towards that end, we are working in three primary directions:

    1. Facilitating applications of Scenic in both existing and new domains.
    2. Creating infrastructure to support the use and development of Scenic.
    3. Building a user and developer community through dissemination and outreach activities.


Application Development
-----------------------

This thrust comprises work to facilitate the use of Scenic in specific application domains, both those where Scenic is already being successfully used and new domains that could have high impact. We are currently focusing on three domains: autonomous driving, robotics, and extended (virtual/augmented) reality.

Autonomous Driving
~~~~~~~~~~~~~~~~~~

The Autonomous Driving Working Group is charged with supporting and expanding Scenic's proven use for safety testing/verification of autonomous vehicles. Planned work includes:

    * Test suite generation
    * Metrics and visualization
    * Improved driver modeling
    * Tutorials on testing autonomous vehicles using Scenic


Robotics
~~~~~~~~

We are working to expand preliminary applications of Scenic to testing and training robotic systems, particularly those which interact with human beings. Planned work includes:

    * Interfaces to simulators including MuJoCo, Gazebo, Habitat, and Isaac Sim
    * A Gym-style API to facilitate training RL agents using Scenic
    * Tutorials on testing and training robots using Scenic


Extended Reality
~~~~~~~~~~~~~~~~

Extended (virtual/augmented) reality is a relatively new application domain for Scenic that we have been exploring. Planned work aims to develop personalized training and evaluation methods for sports and healthcare applications.


Infrastructure Development
--------------------------

This thrust comprises work on computational infrastructure to support Scenic's development. Planned work includes:

    * Enhancing the CI system to test all supported simulators (CARLA and Webots already completed)
    * Enhancing the CI system to benchmark scene generation
    * Creating a system for managing Scenic Improvement Proposals (SIPs)
    * Creating an index for Scenic scenarios and libraries similar to the Python Package Index (PyPI)


Governance and Community Engagement
-----------------------------------

This thrust comprises work to support and grow the community of Scenic users and developers, as well as to develop governance policies ensuring that the evolution of the project reflects the needs of all stakeholders. Planned work includes:

    * Convening working groups for each of the application areas above
    * Developing governance policies, e.g. for electing Steering Committee and Core Team members and for evaluating Scenic Improvement Proposals
    * Continuing and expanding the annual Scenic Workshop
    * Running tutorials at academic and industry conferences
