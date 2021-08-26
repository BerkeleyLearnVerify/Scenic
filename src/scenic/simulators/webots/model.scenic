"""Generic Scenic world model for the Webots simulator."""

def _errorMsg():
    raise RuntimeError('scenario must be run from inside Webots')
simulator _errorMsg()

class WebotsObject:
    """Abstract class for Webots objects.

    The ``webotsName`` property must be set to the name of the Webots node to
    use for this object, which must already exist in the world loaded into
    Webots.

    Also defines the ``elevation`` property as a standard way to access the Y
    component of an object's position, since the Scenic built-in property
    ``position`` is only 2D. If ``elevation`` is set to :obj:`None`, it will be
    updated to the object's Y coordinate in Webots when the simulation starts.

    Properties:
        elevation (float or None; dynamic): default ``None`` (see above).
        requireVisible (bool): Default value ``False`` (overriding the default
            from `Object`).
        webotsName (str): 'DEF' name of the Webots node to use for this object.
        webotsObject: Is set at runtime to a handle to the Webots node for the
            object, for use with the `Supervisor API`_. Primarily for internal
            use.
        controller (str or None): name of the Webots controller to use for
            this object, if any (instead of a Scenic behavior).
        resetController (bool): Whether to restart the controller for each
            simulation (default ``False``).
        positionOffset (`Vector`): Offset to add when computing the object's
            position in Webots; for objects whose Webots ``translation`` field
            is not aligned with the center of the object.

    .. _Supervisor API: https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python
    """

    elevation[dynamic]: None
    requireVisible: False

    webotsName: None
    webotsType: None
    webotsObject: None

    controller: None
    resetController: False

    positionOffset: (0, 0)
