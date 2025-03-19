import os

class CartesianPoint:
    def __init__(self, x=0, y=0, z=0, a=0, b=0, c=0, e1=0, e2=0, e3=0, e4=0):
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
    def __init__(self, a1=0, a2=0, a3=0, a4=0, a5=0, a6=0, e1=0, e2=0, e3=0, e4=0):
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
    def __init__(self, x=0, y=0, z=0, a=0, b=0, c=0):
        self.x = x
        self.y = y
        self.z = z
        self.a = a
        self.b = b
        self.c = c

class SrcGenerator:
    def __init__(self, fName, loc='./', ADV=3, homePos=JointPoint(a2=-90, a3=90, a5=90)):
        self.fName = fName
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
        self.src.write(
            f'LIN {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} C_DIS\n'
        )

    def ptpMotion(self, p=CartesianPoint(), useLinSmoothing=True):
        smoothingArg = 'C_DIS' if useLinSmoothing else 'C_PTP'
        self.src.write(
            f'PTP {{X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} {smoothingArg}\n'
        )

    def jointMotion(self, p=JointPoint()):
        self.src.write(
            f'PTP {{A1 {p.a1}, A2 {p.a2}, A3 {p.a3}, A4 {p.a4}, A5 {p.a5}, A6 {p.a6}, '
            f'E1 {p.e1}, E2 {p.e2}, E3 {p.e3}, E4 {p.e4}}} C_PTP\n'
        )

    def setBaseByPosition(self, p=Position()):
        self.src.write(
            f'$BASE = {{FRAME: X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}}}\n'
        )

    def setToolByPosition(self, p=Position()):
        self.src.write(
            f'$TOOL = {{FRAME: X {p.x}, Y {p.y}, Z {p.z}, A {p.a}, B {p.b}, C {p.c}}}\n'
        )

    def setBaseByID(self, ID=0):
        self.src.write(f'BAS(#BASE, {ID})\n')

    def setToolByID(self, ID=0):
        self.src.write(f'BAS(#TOOL, {ID})\n')

    def delay(self, t=0):  # SECONDS!
        self.src.write(f'WAIT SEC {t}\n')

    def setDOut(self, ID=0, value='FALSE'):
        self.src.write(f'$OUT[{ID}] = {value}\n')

    def setAOut(self, ID=0, value=0.0):
        self.src.write(f'$ANOUT[{ID}] = {value}\n')

    # Linear speed in m/sec
    def setLinSpeed(self, speed=0.25):
        self.src.write(f'BAS(#VEL_CP, {speed})\n')

    # Joint speed in % of max joint speed
    def setJointSpeed(self, speed=30):
        self.src.write(f'BAS(#VEL_PTP, {speed})\n')

    def setLinSmooth(self, val=0):
        self.src.write(f'$APO.CDIS = {val}\n')

    # First six entries are robot joints, the remaining values are external axes
    def setJointSmooth(self, vals=None):
        if vals is None:
            vals = [0] * 12
        for i, val in enumerate(vals):
            self.src.write(f'$APO_DIS_PTP[{i+1}] = {val}\n')

    def setBAS(self, varName='', varValue=0):
        self.src.write(f'BAS({varName}, {varValue})\n')

    def writeFreeKRL(self, content=''):
        self.src.write(content)

    def close(self):
        self.src.write('\nEND\n')
        self.src.close()