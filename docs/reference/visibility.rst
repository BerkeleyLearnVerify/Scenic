
.. _visibility:

*****************
Visibility System
*****************

The Scenic visibility system is composed of two main parts: visibility checks and visible regions.

=================
Visibility Checks
=================

It is often useful to determine whether something is visible from another object, i.e. a visibility check. Scenic performs such checks using ray tracing, accounting for occlusion. Something is considered visible if any ray collides with it, without colliding with an occluding object first. Since Scenic sends a finite number of rays, it is possible for false negatives to occur, though this can be tuned using the properties below. Visibility checks are used by various specifiers and operators, such as the :keyword:`can see` operator, the :keyword:`visible <visible_spec>` specifier, etc.

Various object properties directly affect how Scenic performs visibility checks:

 * :prop:`viewAngles` : The horizontal and vertical angles (in radians) of the object's field of view. The horizontal view
   angle must be between 0 and 2π and the vertical view angle must be between 0 and π.

 * :prop:`visibleDistance`: Distance used to determine the visible range of the object.

 * :prop:`viewRayDensity`: By default determines the number of rays used during visibility checks. This value is the density of rays per
   degree of visible range in one dimension. The total number of rays sent will be this value squared per square degree of this object's
   view angles. This value determines the default value for :prop:`viewRayCount`, so if :prop:`viewRayCount` is overwritten this value is ignored.

 * :prop:`viewRayCount`: The total number of horizontal and vertical view angles to be sent, or None if this value should be computed
   automatically.

 * :prop:`viewRayDistanceScaling`: Whether or not the number of rays should scale with the distance to the object. Ignored if 
   :prop:`viewRayCount` is passed.

 * :prop:`occluding`: Whether or not this object occludes visibility.

Scenic uses several internal heuristics to speed up visibility checks, such as only sending rays where an object might actually be visible. Even with these heuristics, certain types of checks, such as those where an object is fully occluded but would otherwise be visible, can be very expensive. We recommend tuning :prop:`viewRayDensity` if runtimes are problematic, though note this may increase the risk of false negatives. Setting :prop:`viewRayDistanceScaling` to ``True`` can also help, especially in situations where objects can be very far away or very close, but one wishes to avoid setting :prop:`viewRayDensity` to a higher value. If one is seeking to emulate a specific camera resolution, one might instead wish to directly set :prop:`viewRayCount` (e.g. setting it to (1920, 1080) to emulate a full HD camera).

===============
Visible Regions
===============

All Scenic objects define a *visible region*, a `Region` that is "visible" from a given `Object`. Visible regions are not directly used to determine if something is visible from an object: instead they serve an accessory role (e.g. making sampling more efficient). The visible region of a `Point` is a sphere, while that of an `OrientedPoint` or `Object` can be a variety of shapes (see `ViewRegion` for details). An object's visible region is used by various specifiers and operators, such as the :keyword:`visible {region}` operator, the :keyword:`visible <visible_spec>` specifier, etc. Note that while Scenic takes occlusion by other objects into account when testing visibility, the visible region itself ignores occlusion. 
