# Behavior Prediction Scenario Descriptions

This folder includes a library of Scenic programs written for use with the CARLA simulator, mostly based on [NHTSA's Pre-Crash Scenarios](https://rosap.ntl.bts.gov/view/dot/41932/dot_41932_DS1.pdf).

For questions and concerns, please contact Francis Indaheng at <findaheng@berkeley.edu> or post an issue to this repo.

*Note:* These scenarios require [VerifAI](https://verifai.readthedocs.io/) to be installed, since they use VerifAI's Halton sampler by default (the sampler type can be configured as explained [here](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.core.external_params.html): for example, you can add `--param verifaiSamplerType random` when running Scenic to use random sampling instead).


## Intersection

1.  Ego vehicle goes straight at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from opposite lane makes a left turn.
2.  Ego vehicle makes a left turn at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from opposite lane goes straight.
3.  Ego vehicle either goes straight or makes a left turn at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from lateral lane continues straight.
4.  Ego vehicle either goes straight or makes a left turn at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from lateral lane makes a left turn.
5.  Ego vehicle makes a right turn at 4-way intersection while adversary vehicle from opposite lane makes a left turn.
6.  Ego vehicle makes a right turn at 4-way intersection while adversary vehicle from lateral lane goes straight.
7.  Ego vehicle makes a left turn at 3-way intersection and must suddenly stop to avoid collision when adversary vehicle from lateral lane continues straight.
8.  Ego vehicle goes straight at 3-way intersection and must suddenly stop to avoid collision when adversary vehicle makes a left turn.
9.  Ego vehicle makes a right turn at 3-way intersection while adversary vehicle from lateral lane goes straight.
10. Ego Vehicle waits at 4-way intersection while adversary vehicle in adjacent lane passes before performing a lane change to bypass a stationary vehicle waiting to make a left turn.

## Bypassing

1.  Ego vehicle performs a lane change to bypass a slow adversary vehicle before returning to its original lane.
2.  Advesary vehicle performs a lange change to bypass the ego vehicle before returning to its original lane.
3.  Ego vehicle performs a lane change to bypass a slow adversary vehicle but cannot return to its original lane because the adversary accelerates. Ego vehicle must then slow down to avoid collision with leading vehicle in new lane.
4.  Ego vehicle performs multiple lane changes to bypass two slow adversary vehicles.
5.  Ego vehicle performs multiple lane changes to bypass three slow adversary vehicles.

## Pedestrian

1.  Ego vehicle must suddenly stop to avoid collision when pedestrian crosses the road unexpectedly.
2.  Both ego and adversary vehicles must suddenly stop to avoid collision when pedestrian crosses the road unexpectedly.
3.  Ego vehicle makes a left turn at an intersection and must suddenly stop to avoid collision when pedestrian crosses the crosswalk.
4.  Ego vehicle makes a right turn at an intersection and must yield when pedestrian crosses the crosswalk.
5.  Ego vehicle goes straight at an intersection and must yield when pedestrian crosses the crosswalk.
