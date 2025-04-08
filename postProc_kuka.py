import os

class CartesianPoint:
    """
    Represents a Cartesian point in 3D space with orientation and external axes.
    """
    def __init__(self, x=0, y=0, z=0, a=0, b=0, c=0, e1=0, e2=0, e3=0, e4=0):
        """
        Initialize a CartesianPoint with given coordinates and orientation.

        :param x: X coordinate.
        :param y: Y coordinate.
        :param z: Z coordinate.
        :param a: Orientation angle A.
        :param b: Orientation angle B.
        :param c: Orientation angle C.
        :param e1: External axis 1.
        :param e2: External axis 2.
        :param e3: External axis 3.
        :param e4: External axis 4.
        """
        self.x = x
        self.y = y
        self.z = z
        self.a = a
        self.b = b
        self.c = c
        self.e1 = e1
        self.e2 = e2
        self.e3 = e3
        self.e4 = e4

class JointPoint:
    """
    Represents a point defined by joint angles and external axes.
    """
    def __init__(self, a1=0, a2=0, a3=0, a4=0, a5=0, a6=0, e1=0, e2=0, e3=0, e4=0):
        """
        Initialize a JointPoint with given joint angles and external axes.

        :param a1: Joint angle 1.
        :param a2: Joint angle 2.
        :param a3: Joint angle 3.
        :param a4: Joint angle 4.
        :param a5: Joint angle 5.
        :param a6: Joint angle 6.
        :param e1: External axis 1.
        :param e2: External axis 2.
        :param e3: External axis 3.
        :param e4: External axis 4.
        """
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.a5 = a5
        self.a6 = a6
        self.e1 = e1
        self.e2 = e2
        self.e3 = e3
        self.e4 = e4

class Position:
    """
    Represents a position defined by Cartesian coordinates and orientation.
    """
    def __init__(self, x=0, y=0, z=0, a=0, b=0, c=0):
        """
        Initialize a Position with given coordinates and orientation.

        :param x: X coordinate.
        :param y: Y coordinate.
        :param z: Z coordinate.
        :param a: Orientation angle A.
        :param b: Orientation angle B.
        :param c: Orientation angle C.
        """
        self.x = x
        self.y = y
        self.z = z
        self.a = a
        self.b = b
        self.c = c

class SrcGenerator:
    """
    A class to generate KUKA KRL source (.src) files.

    This class provides methods to write various motion commands and settings
    to a source file, including Cartesian motions, joint motions, and specialized
    commands such as SLIN, SPTP, and SPLINE motions.
    """
    def __init__(self, fName, loc='./', ADV=3, homePos=JointPoint(a2=-90, a3=90, a5=90)):
        """
        Initialize the SrcGenerator with a file name, location, advance run value, and home position.

        :param fName: The base name for the source file.
        :param loc: The directory where the source file will be created.
        :param ADV: The advance run value ($ADVANCE) for the robot.
        :param homePos: A JointPoint representing the home position of the robot.
        """
        self.fName = fName
        self.homePos = homePos
        self.src = open(os.path.join(loc, fName + '.src'), 'w')

        self.src.write('&ACCESS RVP\n')
        self.src.write('&REL 1\n')
        self.src.write('&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe\n')
        self.src.write('&PARAM EDITMASK = *\n')
        self.src.write(f'DEF {fName.upper()} ( )\n\n\n')

        # Copy of kukaPRC outputs
        self.src.write(
            ';FOLD INI\n'
            ';FOLD BASISTECH INI\n'
            'GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )\n'
            'INTERRUPT ON 3\n'
            'BAS (#INITMOV,0 )\n'
            ';ENDFOLD (BASISTECH INI)\n'
            ';ENDFOLD (INI)\n\n'
            ';FOLD STARTPOSITION - BASE IS 0, TOOL IS 0, SPEED IS 100%, POSITION IS externally defined -> se generating script\n'
            '$BWDSTART = FALSE\n'
            'PDAT_ACT = {VEL 100,ACC 100,APO_DIST 50}\n'
            'FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}\n'
            'BAS (#PTP_PARAMS,100)\n'
        )

        self.src.write(
            f'PTP {{A1 {homePos.a1}, A2 {homePos.a2}, A3 {homePos.a3}, '
            f'A4 {homePos.a4}, A5 {homePos.a5}, A6 {homePos.a6}, '
            f'E1 {homePos.e1}, E2 {homePos.e2}, E3 {homePos.e3}, E4 {homePos.e4}}}\n;ENDFOLD\n\n'
        )

        self.src.write(f'$ADVANCE = {ADV}\n\n')

    def linMotion(self, p=CartesianPoint()):
        """
        Write a linear (LIN) motion command to the source file.

        :param p: A CartesianPoint object representing the target position.
        """
        self.src.write(
            f'LIN {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} C_DIS\n'
        )

    def ptpMotion(self, p=CartesianPoint(), useLinSmoothing=True):
        """
        Write a point-to-point (PTP) motion command to the source file.

        :param p: A CartesianPoint object representing the target position.
        :param useLinSmoothing: If True, use linear smoothing (C_DIS); otherwise, use point-to-point smoothing (C_PTP).
        """
        smoothingArg = 'C_DIS' if useLinSmoothing else 'C_PTP'
        self.src.write(
            f'PTP {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} {smoothingArg}\n'
        )

    def jointMotion(self, p=JointPoint()):
        """
        Write a joint-based motion command to the source file.

        :param p: A JointPoint object representing the target joint positions.
        """
        self.src.write(
            f'PTP {{A1 {p.a1}, A2 {p.a2}, A3 {p.a3}, A4 {p.a4}, A5 {p.a5}, A6 {p.a6}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} C_PTP\n'
        )

    def slinMotion(self, p=CartesianPoint()):
        """
        Write a spline linear (SLIN) motion command to the source file.
        SLIN uses spline interpolation for smoother motion along a linear path.

        :param p: A CartesianPoint object representing the target position.
        """
        self.src.write(
            f'SLIN {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} C_DIS\n'
        )

    def sptpMotion(self, p=CartesianPoint(), useLinSmoothing=True):
        """
        Write a spline point-to-point (SPTP) motion command to the source file.
        SPTP combines point-to-point accuracy with the smooth transitions of spline interpolation.

        :param p: A CartesianPoint object representing the target position.
        :param useLinSmoothing: If True, use linear smoothing (C_DIS); otherwise, use point-to-point smoothing (C_PTP).
        """
        smoothingArg = 'C_DIS' if useLinSmoothing else 'C_PTP'
        self.src.write(
            f'SPTP {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} {smoothingArg}\n'
        )

    def splineMotion(self, points):
        """
        Write a spline motion block to the source file.
        A spline block smoothly interpolates through a sequence of points.

        :param points: A list of CartesianPoint objects representing the spline path.
        """
        self.src.write("SPLINE\n")
        for p in points:
            self.src.write(
                f"   {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, "
                f"E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} C_DIS\n"
            )
        self.src.write("ENDSPLINE\n")

    def homeMotion(self):
        """
        Write a PTP motion to homeposition to the source file.

        :param homepos: Cartesian Point with coordinates of robot homeposition
        """
        self.src.write(
            f'PTP {{A1 {self.homePos.a1}, A2 {self.homePos.a2}, A3 {self.homePos.a3}, '
            f'A4 {self.homePos.a4}, A5 {self.homePos.a5}, A6 {self.homePos.a6}, '
            f'E1 {self.homePos.e1}, E2 {self.homePos.e2}, E3 {self.homePos.e3}, E4 {self.homePos.e4}}}\n;ENDFOLD\n\n'
        )

    def setBaseByPosition(self, p=Position()):
        """
        Set the base coordinate system using a Position.

        :param p: A Position object representing the desired base frame.
        """
        self.src.write(
            f'$BASE = {{FRAME: X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}}}\n'
        )

    def setToolByPosition(self, p=Position()):
        """
        Set the tool coordinate system using a Position.

        :param p: A Position object representing the desired tool frame.
        """
        self.src.write(
            f'$TOOL = {{FRAME: X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}}}\n'
        )

    def setBaseByID(self, ID=0):
        """
        Set the base coordinate system by its ID.

        :param ID: An integer representing the base ID.
        """
        self.src.write(f'BAS(#BASE, {ID})\n')

    def setToolByID(self, ID=0):
        """
        Set the tool coordinate system by its ID.

        :param ID: An integer representing the tool ID.
        """
        self.src.write(f'BAS(#TOOL, {ID})\n')

    def vorfraesen(self):
        """
        Write a Vorfraesen command to the source file.
        Vorfraesen is a KUKA command for pre-processing an outline of the drilling before the actual drilling process.
        """
        self.src.write("Vorfraesen()\n")

    def delay(self, t=0):
        """
        Insert a delay (WAIT) command into the source file.

        :param t: Time in seconds to wait.
        """
        self.src.write(f'WAIT SEC {t}\n')

    def setDOut(self, ID=0, value='FALSE'):
        """
        Set a digital output to a specified value.

        :param ID: The digital output ID.
        :param value: The value to set ('TRUE' or 'FALSE').
        """
        self.src.write(f'$OUT[{ID}] = {value}\n')

    def setAOut(self, ID=0, value=0.0):
        """
        Set an analog output to a specified value.

        :param ID: The analog output ID.
        :param value: The value to set (typically a float).
        """
        self.src.write(f'$ANOUT[{ID}] = {value}\n')

    def setLinSpeed(self, speed=0.25):
        """
        Set the linear speed for Cartesian motions.

        :param speed: Speed in meters per second.
        """
        self.src.write(f'BAS(#VEL_CP, {speed})\n')

    def setJointSpeed(self, speed=30):
        """
        Set the joint speed as a percentage of maximum joint speed.

        :param speed: Speed as a percentage (0-100).
        """
        self.src.write(f'BAS(#VEL_PTP, {speed})\n')

    def setLinSmooth(self, val=0):
        """
        Set the linear smoothing parameter for CP motions.

        :param val: The smoothing value (usually in mm).
        """
        self.src.write(f'$APO.CDIS = {val}\n')

    def setJointSmooth(self, vals=None):
        """
        Set the smoothing parameters for joint motions.

        :param vals: A list of smoothing values for the joints and external axes.
                     Defaults to a list of 12 zeros if not provided.
        """
        if vals is None:
            vals = [0] * 12
        for i, val in enumerate(vals):
            self.src.write(f'$APO_DIS_PTP[{i+1}] = {val}\n')

    def setBAS(self, varName='', varValue=0):
        """
        Write a BAS command to set a parameter.

        :param varName: The name of the BAS variable.
        :param varValue: The value to set for the variable.
        """
        self.src.write(f'BAS({varName}, {varValue})\n')

    def writeFreeKRL(self, content=''):
        """
        Write arbitrary KRL code directly into the source file.

        :param content: The free-form KRL code to write.
        """
        self.src.write(content)

    def close(self):
        """
        Finalize and close the source file.
        This writes the END command and closes the file.
        """
        self.src.write('\nEND\n')
        self.src.close()
