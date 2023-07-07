import pytest

import scenic
from scenic.core.distributions import RejectionException
from scenic.core.errors import InvalidScenarioError, ScenicSyntaxError
from tests.utils import compileScenic, sampleEgo, sampleScene, sampleSceneFrom

## Basic


def test_requirement():
    scenario = compileScenic(
        """
        ego = new Object at Range(-10, 10) @ 0
        require ego.position.x >= 0
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(0 <= x <= 10 for x in xs)


def test_soft_requirement():
    scenario = compileScenic(
        """
        ego = new Object at Range(-10, 10) @ 0
        require[0.9] ego.position.x >= 0
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(350)]
    count = sum(x >= 0 for x in xs)
    assert 255 <= count < 350


def test_illegal_soft_probability():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            ego = new Object
            require[1.1] ego.position.x >= 0
        """
        )


def test_named_requirement():
    scenario = compileScenic(
        """
        ego = new Object at Range(0, 10) @ 0
        require ego.position.x >= 5 as posReq
        require True as 'myReq'
        require True as 101
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(5 <= x <= 10 for x in xs)


@pytest.mark.slow
def test_named_soft_requirement():
    scenario = compileScenic(
        """
        ego = new Object at Range(0, 10) @ 0
        require[0.9] ego.position.x >= 5 as posReq
        require[0.8] True as 'myReq'
        require[0.75] True as 101
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(350)]
    count = sum(x >= 5 for x in xs)
    assert 255 <= count < 350


def test_named_requirement_invalid():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            ego = new Object
            require True as +
        """
        )


def test_unexpected_keyword_arg():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            ego = new Object
            require True, line=5
        """
        )


def test_unexpected_unpacking():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            ego = new Object
            a = (True,)
            require *a
        """
        )


## Forbidden operations inside requirements


def test_distribution_in_requirement():
    scenario = compileScenic(
        """
        require Range(0, 1) <= 1
        ego = new Object
    """
    )
    with pytest.raises(InvalidScenarioError):
        sampleScene(scenario)


def test_object_in_requirement():
    scenario = compileScenic(
        """
        require new Object
        ego = new Object
    """
    )
    with pytest.raises(InvalidScenarioError):
        sampleScene(scenario)


def test_param_in_requirement_1():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            require param x = 4
            ego = new Object
        """
        )


def test_param_in_requirement_2():
    with pytest.raises(InvalidScenarioError):
        scenario = compileScenic(
            """
            def func():
                param x = 4
                return True
            require func()
            ego = new Object
        """
        )
        sampleScene(scenario)


@pytest.mark.xfail(reason="looser keyword policy now allows this", strict=True)
def test_mutate_in_requirement_1():
    scenario = compileScenic(
        """
        require mutate
        ego = new Object
    """
    )
    with pytest.raises(ScenicSyntaxError):
        sampleScene(scenario)


def test_mutate_in_requirement_2():
    with pytest.raises(InvalidScenarioError):
        scenario = compileScenic(
            """
            def func():
                mutate ego
                return True
            require func()
            ego = new Object
        """
        )
        sampleScene(scenario)


def test_require_in_requirement():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            require (require True)
            ego = new Object
        """
        )


## Error handling


def test_exception_in_requirement():
    scenario = compileScenic(
        """
        require visible 4
        ego = new Object
    """
    )
    with pytest.raises(TypeError):
        sampleScene(scenario)


def test_soft_requirement_with_temporal_operators():
    """Temporal operators are not allowed if prob != 1"""
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            ego = new Object
            require[0.2] eventually ego
        """
        )


## Enforcement of built-in requirements


def test_containment_requirement():
    scenario = compileScenic(
        """
        foo = RectangularRegion(0@0, 0, 10, 10)
        ego = new Object at Range(0, 10) @ 0, with regionContainedIn foo
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(0 <= x <= 5 for x in xs)


def test_containment_workspace():
    scenario = compileScenic(
        """
        workspace = Workspace(RectangularRegion(0@0, 0, 10, 10))
        ego = new Object at Range(0, 10) @ 0
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(0 <= x <= 5 for x in xs)


def test_visibility_requirement():
    scenario = compileScenic(
        """
        ego = new Object with visibleDistance 10, with viewAngle 90 deg, facing 45 deg
        other = new Object at Range(-10, 10) @ 0, with requireVisible True
    """
    )
    xs = [
        sampleScene(scenario, maxIterations=60).objects[1].position.x for i in range(60)
    ]
    assert all(-10 <= x <= 0.5 for x in xs)


def test_visibility_requirement_disabled():
    scenario = compileScenic(
        """
        ego = new Object with visibleDistance 10, with viewAngle 90 deg, facing 45 deg
        other = new Object at Range(-10, 10) @ 0, with requireVisible False
    """
    )
    xs = [
        sampleScene(scenario, maxIterations=60).objects[1].position.x for i in range(60)
    ]
    assert any(x > 0.5 for x in xs)


def test_intersection_requirement():
    scenario = compileScenic(
        """
        ego = new Object at Range(0, 2) @ 0
        other = new Object
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(x >= 1 for x in xs)


def test_intersection_requirement_disabled_1():
    scenario = compileScenic(
        """
        ego = new Object at Range(0, 2) @ 0, with allowCollisions True
        other = new Object
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert any(x < 1 for x in xs)


def test_intersection_requirement_disabled_2():
    scenario = compileScenic(
        """
        ego = new Object at Range(0, 2) @ 0
        other = new Object with allowCollisions True
    """
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert any(x < 1 for x in xs)


## Static violations of built-in requirements


def test_static_containment_violation():
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            foo = RectangularRegion(0@0, 0, 5, 5)
            ego = new Object at 10@10, with regionContainedIn foo
        """
        )


def test_static_containment_workspace():
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            workspace = Workspace(RectangularRegion(0@0, 0, 5, 5))
            ego = new Object at 10@10
        """
        )


def test_static_empty_container():
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            foo = PolylineRegion([0@0, 1@1]).intersect(PolylineRegion([1@0, 2@1]))
            ego = new Object at Range(0, 2) @ Range(0, 1), with regionContainedIn foo
        """
        )


def test_static_visibility_violation_enabled():
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            ego = new Object at 10@0, facing -90 deg, with viewAngle 90 deg
            new Object at 0@10, with requireVisible True
        """
        )


def test_static_visibility_violation_enabled_2d():
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            ego = new Object at 10@0, facing -90 deg, with viewAngle 90 deg
            new Object at 0@10, with requireVisible True
        """,
            mode2D=True,
        )


def test_static_visibility_violation_disabled():
    sampleSceneFrom(
        """
        ego = new Object at 10@0, facing -90 deg, with viewAngle 90 deg
        new Object at 0@10, with requireVisible False
    """
    )


def test_static_intersection_violation():
    with pytest.raises(InvalidScenarioError):
        compileScenic(
            """
            ego = new Object at 0@0
            new Object at 0.5@0
        """
        )


def test_static_intersection_violation_disabled():
    sampleSceneFrom(
        """
        ego = new Object at 0@0
        new Object at 1@0, with allowCollisions True
    """
    )


# Occlusion visibility requirements


@pytest.mark.slow
def test_can_see_object_occlusion_enabled():
    with pytest.raises(RejectionException):
        sampleSceneFrom(
            """
            workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
            workspace = Workspace(workspace_region)

            ego = new Object with visibleDistance 30,
                at (0,0,1),
                with width 5,
                with length 5,
                with height 5,
                with pitch 45 deg,
                with viewAngles (340 deg, 60 deg),
                with rayDensity 5

            seeing_obj = new Object at (0,10,5),
                with width 2,
                with height 2,
                with length 2,
                with name "seeingObject",
                with requireVisible True

            new Object at (0,5,4),
                with width 10,
                with length 0.5,
                with height 6,
                with name "wall",
        """,
            maxIterations=1,
        )


@pytest.mark.slow
def test_can_see_object_occlusion_disabled():
    sampleSceneFrom(
        """
        workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
        workspace = Workspace(workspace_region)

        ego = new Object with visibleDistance 30,
            at (0,0,1),
            with width 5,
            with length 5,
            with height 5,
            with pitch 45 deg,
            with viewAngles (340 deg, 60 deg),
            with rayDensity 5

        seeing_obj = new Object at (0,10,5),
            with width 2,
            with height 2,
            with length 2,
            with name "seeingObject",
            with requireVisible True

        new Object at (0,5,4),
            with width 10,
            with length 0.5,
            with height 6,
            with name "wall",
            with occluding False
    """,
        maxIterations=1,
    )


## Tests for random properties that have special effects on constructing requirements
def test_random_allowCollisions():
    scene = sampleSceneFrom(
        """
        new Object with allowCollisions Uniform(True, False)
        new Object with allowCollisions Uniform(True, False)
    """,
        maxIterations=30,
    )

    assert any(obj.allowCollisions for obj in scene.objects)


@pytest.mark.slow
def test_random_occlusion():
    scene = sampleSceneFrom(
        """
        workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
        workspace = Workspace(workspace_region)

        ego = new Object with visibleDistance 30,
            at (0,0,1),
            with width 5,
            with length 5,
            with height 5,
            with pitch 45 deg,
            with viewAngles (340 deg, 60 deg),
            with rayDensity 5

        seen_obj = new Object at (0,10,5),
            with width 2,
            with height 2,
            with length 2,
            with name "seen_obj",
            with requireVisible True

        new Object at (0,5,4),
            with width 10,
            with length 0.5,
            with height 6,
            with name "wall",
            with occluding Uniform(True, False)
    """,
        maxIterations=60,
    )

    assert any(
        hasattr(obj, "name") and obj.name == "wall" and (not obj.occluding)
        for obj in scene.objects
    )
