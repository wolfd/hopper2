import math

from xml.etree.ElementTree import Element, SubElement, tostring

class RobotLink(object):
    def __init__(self, root, name):
        self.name = name
        self.xml = SubElement(root, "link", {"name": name})

        self.visual = None
        self.collision = None

    def _create_visual(self):
        return SubElement(self.xml, "visual")

    def _create_collision(self):
        return SubElement(self.xml, "collision")

    def _create_inertial(self, mass, inertia):
        inertial = SubElement(self.xml, "inertial")
        SubElement(inertial, "mass", {"value": str(mass)})
        # fix
        SubElement(inertial, "inertia", inertia)
        return inertial

    def make_box(self, length, width, height, mass):
        self.visual = self._create_visual()
        box_geometry(self.visual, length, width, height)
        default_material(self.visual)

        self.collision = self._create_collision()
        box_geometry(self.collision, length, width, height)

        self._create_inertial(
            mass,
            box_inertia(length, width, height, mass)
        )

    def make_cylinder(self, radius, length, mass):
        self.visual = self._create_visual()
        cylinder_geometry(self.visual, radius, length)
        default_material(self.visual)

        self.collision = self._create_collision()
        cylinder_geometry(self.collision, radius, length)

        self._create_inertial(
            mass,
            cylinder_inertia(radius, length, mass)
        )


class RobotWheel(RobotLink):
    def __init__(self, root, name, radius, length):
        super(RobotWheel, self).__init__(root, name)
        self.radius = radius
        self.length = length


def box_geometry(parent, length, width, height):
    geometry = SubElement(parent, "geometry")
    SubElement(geometry, "box", {"size": "{} {} {}".format(length, width, height)})
    return geometry

def cylinder_geometry(parent, radius, length):
    geometry = SubElement(parent, "geometry")
    SubElement(geometry, "cylinder", {
        "radius": "{}".format(radius),
        "length": "{}".format(length)
    })
    return geometry

# z up
def turn_z_up_toward_axis(visual_or_collision, axis, angle=math.pi / 2):
    x = 0
    y = 0
    z = 0
    if axis == "x":
        y = angle
    elif axis == "y":
        x = angle
    elif axis == "z":
        z = angle
        # ?

    return SubElement(visual_or_collision, "origin", {
        "rpy": "{} {} {}".format(x, y, z),
        "xyz": "0 0 0"
    })

def point_z_left(visual_or_collision):
    SubElement(visual_or_collision, "origin", {
        "rpy": "{} {} {}".format(math.pi / 2, 0, 0),
        "xyz": "0 0 0"
    })

# length = x, width = y, height = z
def box_inertia(length, width, height, mass):
    return {
        "ixx": str(1.0/12.0 * mass * (width ** 2 + height ** 2)),
        "iyy": str(1.0/12.0 * mass * (length ** 2 + height ** 2)),
        "izz": str(1.0/12.0 * mass * (length ** 2 + width ** 2))
    }

# z-axis aligned
def cylinder_inertia(radius, length, mass):
    return {
        "ixx": str(1.0/12.0 * mass * (3.0 * radius ** 2 + length ** 2)),
        "iyy": str(1.0/12.0 * mass * (3.0 * radius ** 2 + length ** 2)),
        "izz": str(1.0/2.0 * mass * (radius ** 2))
    }

class RobotCar:
    def __init__(self, name, length=25.4 / 100., width=16.51 / 100., height=5.0 / 100.):
        self.root = Element("robot", {"name": name})

        self.length = length
        self.width = width
        self.height = height

        # create main car body
        self.chassis = self.create_link("chassis")

        self.chassis.make_box(length, width, height, mass=0.7)

        wheel_width = 0.02
        front_left_wheel = self.create_wheel("front_left_wheel", length=wheel_width)
        self.create_joint(
            self.chassis,
            front_left_wheel,
            "front_left_link",
            axis="0 1 1",
            origin="{} {} {}".format(
                self.length / 2.0 - front_left_wheel.radius / 2,
                self.width / 2.0 + front_left_wheel.length / 2,
                0
            )
        )

        front_right_wheel = self.create_wheel("front_right_wheel", length=wheel_width)
        self.create_joint(
            self.chassis,
            front_right_wheel,
            "front_right_link",
            axis="0 1 0",
            origin="{} {} {}".format(
                self.length / 2.0 - front_right_wheel.radius / 2,
                -(self.width / 2.0 + front_right_wheel.length / 2),
                0
            )
        )

        back_left_wheel = self.create_wheel("back_left_wheel", length=wheel_width, angle=math.pi / 4)
        self.create_joint(
            self.chassis,
            back_left_wheel,
            "back_left_link",
            axis="0 1 -1",
            origin="{} {} {}".format(
                -(self.length / 2.0 - back_left_wheel.radius / 2),
                self.width / 2.0 + back_left_wheel.length / 2,
                0
            )
        )

        back_right_wheel = self.create_wheel("back_right_wheel", length=wheel_width, angle=math.pi / -4)
        self.create_joint(
            self.chassis,
            back_right_wheel,
            "back_right_link",
            axis="0 1 1",
            origin="{} {} {}".format(
                -(self.length / 2.0 - back_right_wheel.radius / 2),
                -(self.width / 2.0 + back_right_wheel.length / 2),
                0
            )
        )

    def add_flywheel(self):
        flywheel = self.create_wheel(
            "flywheel",
            radius=4.44500 / 100.0,
            length=0.01,
            axis="x"
        )
        # 3.250, 4.547244 inch
        flywheel_x_position = (11.54999976 / 100.0) - self.length / 2.0

        # should be 0
        flywheel_y_position = (8.255 / 100.0) - self.width / 2.0
        assert flywheel_y_position == 0.0

        self.create_joint(
            self.chassis,
            flywheel,
            "flywheel_link",
            axis="1 0 0",
            origin="{} {} {}".format(
                flywheel_x_position,
                flywheel_y_position,
                0
            )
        )

    def create_link(self, name):
        return RobotLink(self.root, name)

    def create_joint(self, parent, child, name, joint_type="continuous", axis="0 1 0", origin="0 0 0"):
        joint = SubElement(self.root, "joint", {"name": name, "type": joint_type})
        SubElement(joint, "parent", {"link": parent.name})
        SubElement(joint, "child", {"link": child.name})
        SubElement(joint, "axis", {"xyz": axis})
        SubElement(joint, "origin", {"xyz": origin})

        return joint

    def create_wheel(self, name, radius=0.051, length=0.02, mass=0.102, axis="y", angle=math.pi / 2):
        base = RobotWheel(self.root, name, radius, length)
        base.make_cylinder(radius, length, mass)

        # our wheels all point left instead of up
        turn_z_up_toward_axis(base.visual, axis, angle)
        turn_z_up_toward_axis(base.collision, axis, angle)

        return base


def default_material(visual):
    material = SubElement(visual, "material", {"name": "white"})
    SubElement(material, "color", {"rgba": "1.0 1.0 1.0 1.0"})
    return material

car_urdf = RobotCar("faller")

with open('models/car.urdf', 'w') as output_f:
    output_f.write(tostring(car_urdf.root))
