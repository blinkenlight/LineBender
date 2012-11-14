# coding: utf-8

# ******************************************************************************
# Copyright © 2012 Asztalos Attila Oszkár
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ******************************************************************************

import os;
import re;
import sys;
import logging;

from math import sqrt, radians, sin, cos, atan;

# ******************************************************************************
# Constants (fine - "variables not to be modified", happy now?)
# ******************************************************************************

VERSION = 0.1;

ONE_MIL = 0.001;
ONE_TENTH_MIL = 0.0001;

UNITS_MODE_METRIC = 1;
UNITS_MODE_IMPERIAL = 2;

COORDS_MODE_ABSOLUTE = 1;
COORDS_MODE_RELATIVE = 2;

WORK_PLANE_XY = 1;
WORK_PLANE_ZX = 2;
WORK_PLANE_YZ = 3;

# ******************************************************************************
# Global variables
# ******************************************************************************

TextLinesHandled = 0;                                                           # Number of lines of g-code analyzed (should match the number of lines of text in the file)
TextLinesIgnored = 0;                                                           # Number of lines of g-code ignored (because no G0/G1/G2/G3 or X/Y/Z was found on the line)

PathLinesHandled = 0;                                                           # Number of path lines analyzed (number of lines found containing a G0/G1/G2/G3)
PathLinesDropped = 0;                                                           # Number of path lines removed as too short (Line Grinder uses these). Not implemented in 0.1
PathArcsAdjusted = 0;                                                           # Number of arc path lines recalculated (all arcs, currently, whether really needed or not)
PathArcsUnedited = 0;                                                           # Number of arc path lines NOT recalculated: any full circles (sole exceptions, see above )

UnitsMode = None;                                                               # Tracks current units mode (metric/imperial). Not implemented in 0.1
CoordsMode = None;                                                              # Tracks current coordinate mode (absolute/relative). Not implemented in 0.1
WorkPlane = None;                                                               # Tracks current work plane (XY/XZ/YZ). Not implemented in 0.1

CurrentPosition = {'X': None, 'Y': None, 'Z': None};                            # This tracks current X/Y/Z position at all times to be used as starting point for arcs

# ******************************************************************************
# sqr() - well what do you think it does?!? Since Python couldn't be bothered...
# ******************************************************************************

def sqr(x):

    return(x*x);

# ******************************************************************************
# dist() - distance between two points A and B
#
# Note: at some point I might consider rewriting all these to collected (X,Y)
# points instead of standalone X's and Y's. Some MORE convenient point in time.
# PCB... waiting... code... faster...!
# ******************************************************************************

def dist(Xa, Ya, Xb, Yb):

    return(sqrt(sqr(Xa - Xb) + sqr(Ya - Yb)));

# ******************************************************************************
# BendThatArc() - does the actual math to recalculate the arc's center
#
# Note: X0,Y0 = arc start, X1,Y1 = arc end, X2,Y2 = original arc center
# Also, the proverbial compulsory bug in every software probably lives in here
#
# To briefly explain what's going on below, try to imagine solving it this way:
# In a custom coordinate system where both arc endpoints are on the "X" axis
# and the "Y" axis passes exactly halfway between them, the solution for the
# center of the arc is trivial: X=0, and Y is found as one side in the right
# angled triangle in which the distance to the arc endpoints is the hypotenuse
# and is fixed as the arc radius, while the other side is half the distance 
# between the endpoints. Once we have this (X, Y) for the center, it's just a 
# matter of (roto-)translating our custom coordinate system onto the actual
# frame of reference - not hard once we find the coordinates of the middle
# point of the segment connecting the arc's endpoints and the angle that
# segment makes with the horizontal ("X"-axis) of the "real" reference frame.
# In fact, some of you might recognize the trigonometric bit towards the end 
# as a specific form of the general equations for transforming coordinate
# systems. There are two possible solutions - we pick the one we like more...
# ******************************************************************************

def BendThatArc(X0, Y0, X1, Y1, X2, Y2):

    R = dist(X0, Y0, X2, Y2);                                                   # Radius we want to keep for the arc (equal to the original arc's radius)
    
    T = dist(X0, Y0, X1, Y1) / 2;                                               # Half of the distance between the arc's endpoints (half length of the segment connecting them)

    S = sqrt(sqr(R) - sqr(T));                                                  # Center's distance from the midpoint on the perpendicular that goes through it (in either direction)
    
    Xm = (X0 + X1) / 2;                                                         # Coordinates of the above-mentioned midpoint of the segment connecting the arc's endpoints
    Ym = (Y0 + Y1) / 2;                                                         # These are our translation coordinates for the reference frame transformation

    if X0 == X1:                                                                # Now we just need the angle between our custom and the global reference frame
        alfa = radians(90);                                                     # We'd rather avoid a division by zero for the alfa = 90 degrees case (endpoints on the same vertical)
    else:
        alfa = atan((Y1 - Y0) / (X1 - X0));                                     # Otherwise, the "slope" of the segment connecting the arc endpoints is the angle we want
                    
    X3 = Xm + S * sin(alfa);                                                    # The actual coordinate system transformation of the point (X=0, Y=S) into the real reference frame
    Y3 = Ym - S * cos(alfa);

    X4 = Xm - S * sin(alfa);                                                    # The other possible solution.
    Y4 = Ym + S * cos(alfa);
    
    if dist(X2, Y2, X3, Y3) < dist(X2, Y2, X4, Y4):                             # Cheating bigtime: there are inherently two centers that would fit two endpoints and a radius.
        return(X3, Y3);                                                         # Finding the proper one would entail figuring out the tangents at the endpoints of the arc        
    else:                                                                       # which in turn would mean looking behind and ahead further for the adjoining segments.
        return(X4, Y4);                                                         # Instead, we simply choose the solution closer to the original center...
        
# ******************************************************************************
# AdjustArc() - recalculate origin (I,J) to match start and end points better
#
# Note: Arcs in non-XY planes...? K...? Heeey, where is everyone going...?!?
# ******************************************************************************

def AdjustArc(Position, Values, Params):

    global PathArcsUnedited;

    Xs = Position['X'];                                                         # Coordinates of the starting point for the arc
    Ys = Position['Y'];

    if Values['X'] is not None:                                                 # Update the arc end point X coordinate if there is one on this line of code,
        Xe = Values['X'];
    else:                                                                       # otherwise just use the current one
        Xe = Xs;
    
    if Values['Y'] is not None:                                                 # Update the arc end point Y coordinate if there is one on this line of code,
        Ye = Values['Y'];
    else:
        Ye = Ys;                                                                # otherwise just use the current one

    if Values['I'] is not None:                                                 # Calculate the absolute arc center X coordinate from the relative I if there is one on this line of code,
        Xc = Xs + Values['I'];
    else:
        Xc = Xs;                                                                # otherwise just use the current X

    if Values['J'] is not None:                                                 # Calculate the absolute arc center Y coordinate from the relative I if there is one on this line of code,
        Yc = Ys + Values['J'];
    else:
        Yc = Ys;                                                                # otherwise just use the current Y

    if  dist(Xs, Ys, Xe, Ye) > 0:                                               # Cannot recalculate full circle arcs from endpoint(s) and radius; thankfully, there's no need either - they're always valid
        NewXc, NewYc = BendThatArc(Xs, Ys, Xe, Ye, Xc, Yc);                     # Do the magic, get some new center coordinates
        
        NewI = NewXc - Xs;                                                      # Calculate the relative I/J members from the absolute center X/Y
        NewJ = NewYc - Ys;
            
        if Params['N'] is None:                                                 # If there is no line number,
            LineNumberString = "";                                              # set the line number string to empty
        else:                                                                   # If there is one,
            LineNumberString = " (aka \"N" + Params['N'] + "\")";               # form a string to be included in any messages referring to this line

        OldCoordString = "[{0:.4f}, {1:.4f}]".format(Xc, Yc);
        NewCoordString = "[{0:.4f}, {1:.4f}]".format(NewXc, NewYc);
            
        logging.debug("Adjusting center of arc from line {0}".format(TextLinesHandled) + LineNumberString + " from " + OldCoordString + " to " + NewCoordString);
            
        CommentString = " (Center moved by Line Bender from " + OldCoordString + " to " + NewCoordString + ")\n";
    else:                                                                       # If this arc is a full circle, we have to skip it
        NewI = Values['I'];                                                     # so the original center is preserved as the "new" one
        NewJ = Values['J'];

        PathArcsUnedited += 1;                                                  # Count this arc as skipped

        if Params['N'] is None:                                                 # If there is no line number,
            LineNumberString = "";                                              # set the line number string to empty
        else:                                                                   # If there is one,
            LineNumberString = " (aka \"N" + Params['N'] + "\")";               # form a string to be included in any messages referring to this line
        
        logging.debug("Preserving center of arc from line {0}".format(TextLinesHandled) + LineNumberString + " - center cannot be determined for full circles");
            
        CommentString = " (Center preserved by Line Bender - center cannot be determined for full circles)\n";
        
    # *** This is also fudged(more corner cutting). It could use a bit of customization depending on which params are present and/or unchanged, no?
    
    NewLine = "G" + Params['G'] + " X{0:.4f} Y{1:.4f} I{2:.4f} J{3:.4f}".format(Xe, Ye, NewI, NewJ) + CommentString;
    
    return(NewLine);                                                            # Return the newly recalculated arc as a text line to replace the old line of g-code

# ******************************************************************************
# ScanForParams() - split the line into code words, return a dictionary of them
#
# Note: Only one instance of each word per line is handled right now even though
# multiple ones are quite legal (but not common in machine-generated g-code);
# The elements in the dictionary are either 'None' or the single string value.
# Also, it's hardly optimal to look for everyting on every line. I know. Sorry.
# ******************************************************************************

def ScanForParams(WorkLine):

    WorkLine = re.sub(";.*", "", WorkLine);                                     # Zap any semicolon-type comments (from semicolon to the end of the line)

    WorkLine = re.sub("\(.*?\)", "", WorkLine);                                 # Yank any parentheses-based comments (only whatever is between any parenthesis sets)
                                                                                # We don't do this because we're evil, but to avoid matching commands inside a comment
    WorkLine = re.sub(" |\t", "", WorkLine);                                    # We also Kick all tabs and spaces (why? BECAUSE WE CAN...!)
                                                                                # Seriously, spaces and tabs are valid anywhere, and might mess up our search logic
    # *** N-words **********************
    
    NWords = re.search("N(\d+)", WorkLine, re.IGNORECASE);                      # Search for a "Nxx" style line number. Only the first occurence, sorry...
    
    if NWords is None:                                                          # If no N-word is found,
        NParam = None;                                                          # set the return value to 'None',
        LineNumberString = "";                                                  # and the line number string to empty
    else:                                                                       # If one is found,
        NParam = NWords.group(1);                                               # store its parameter to be returned,
        LineNumberString = " (aka \"N" + NParam + "\")";                        # and form a string to be included in any messages referring to this line
    
    CommonErrorString = " on line {0}".format(TextLinesHandled) + LineNumberString + ", exiting.";

    # *** G-words **********************
    
    GWords = re.findall("G(\d+)", WorkLine, re.IGNORECASE);                     # Find all "Gxx" style codes (just the values)
    
    if len(GWords) > 1:                                                         # If there are multiple G-words on this line, we're busted, sorry...
        logging.error("Multiple \"G\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(GWords) > 0:                                                       # If there is one, store its parameter to be returned,
        GParam = GWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        GParam = None;
        
    # *** M-words **********************

    MWords = re.findall("M(\d+)", WorkLine, re.IGNORECASE);                     # Find all "Mxx" style codes (just the values)    

    if len(MWords) > 1:                                                         # If there are multiple M-words on this line, we're busted, sorry...
        logging.error("Multiple \"M\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(MWords) > 0:                                                       # If there is one, store its parameter to be returned,
        MParam = MWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        MParam = None;

    # *** F-words **********************

    FWords = re.findall("F([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Fxx" style codes possibly including sign and decimal point (only values)
    
    if len(FWords) > 1:                                                         # If there are multiple F-words on this line, we're busted, sorry...
        logging.error("Multiple \"F\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(FWords) > 0:                                                       # If there is one, store its parameter to be returned,
        FParam = FWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        FParam = None;
    
    # *** X-words **********************
    
    XWords = re.findall("X([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Xxxx" style codes possibly including sign and decimal point (only values)

    if len(XWords) > 1:                                                         # If there are multiple X-words on this line, we're busted, sorry...
        logging.error("Multiple \"X\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(XWords) > 0:                                                       # If there is one, store its parameter to be returned,
        XParam = XWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        XParam = None;

    # *** Y-words **********************

    YWords = re.findall("Y([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Yxxx" style codes possibly including sign and decimal point (only values)

    if len(YWords) > 1:                                                         # If there are multiple Y-words on this line, we're busted, sorry...
        logging.error("Multiple \"Y\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(YWords) > 0:                                                       # If there is one, store its parameter to be returned,
        YParam = YWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        YParam = None;

    # *** Z-words **********************

    ZWords = re.findall("Z([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Zxxx" style codes possibly including sign and decimal point (only values)

    if len(ZWords) > 1:                                                         # If there are multiple Z-words on this line, we're busted, sorry...
        logging.error("Multiple \"Z\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(ZWords) > 0:                                                       # If there is one, store its parameter to be returned,
        ZParam = ZWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        ZParam = None;
            
    # *** I-words **********************

    IWords = re.findall("I([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Ixxx" style codes possibly including sign and decimal point (only values)
    
    if len(IWords) > 1:                                                         # If there are multiple I-words on this line, we're busted, sorry...
        logging.error("Multiple \"I\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(IWords) > 0:                                                       # If there is one, store its parameter to be returned,
        IParam = IWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        IParam = None;

    # *** J-words **********************

    JWords = re.findall("J([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Jxxx" style codes possibly including sign and decimal point (only values)
    
    if len(JWords) > 1:                                                         # If there are multiple J-words on this line, we're busted, sorry...
        logging.error("Multiple \"J\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(JWords) > 0:                                                       # If there is one, store its parameter to be returned,
        JParam = JWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        JParam = None;

    # *** K-words **********************
    
    KWords = re.findall("K([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Kxxx" style codes possibly including sign and decimal point (only values)
    
    if len(KWords) > 1:                                                         # If there are multiple K-words on this line, we're busted, sorry...
        logging.error("Multiple \"K\" words found" + CommonErrorString);
        sys.exit(1);
    elif len(KWords) > 0:                                                       # If there is one, store its parameter to be returned,
        KParam = KWords[0];
    else:                                                                       # If there are none, set the return value accordingly
        KParam = None;
    
    # *** R-words **********************

    RWords = re.findall("R([+-]{,1}[\d.]+)", WorkLine, re.IGNORECASE);          # Find all "Rxxx" style codes possibly including sign and decimal point (only values)

    if len(RWords) > 0:                                                         # If there are any R-words on this line, it's the wrong arc format, sorry...
        logging.error("Radius format arc found" + CommonErrorString);
        sys.exit(1);

    Params = dict({'N': NParam, 'G': GParam, 'M': MParam, 'F': FParam, 'X': XParam, 'Y': YParam, 'Z': ZParam, 'I': IParam, 'J': JParam, 'K': KParam});
    
    return(Params);

# ******************************************************************************
# WordsToValues() - convert relevant string G-word parameters to numeric ones
# ******************************************************************************

def WordsToValues(Params):

    if Params['N'] is None:                                                     # If there is no N-word,
        LineNumberString = "";                                                  # set the line number string to empty
    else:                                                                       # If there is one,
        LineNumberString = " (aka \"N" + Params['N'] + "\")";                   # form a string to be included in any messages referring to this line
    
    CommonErrorString = " on line {0}".format(TextLinesHandled) + LineNumberString + ", exiting.";

    try:
        if Params['G'] is None:                                                 # If it exists, convert the parameter to a number
            GValue = None;
        else:
            GValue = int(Params['G']);
            
        if Params['M'] is None:                                                 # If it exists, convert the parameter to a number
            MValue = None;
        else:
            MValue = int(Params['M']);
            
        if Params['X'] is None:                                                 # If it exists, convert the parameter to a number
            XValue = None;
        else:
            XValue = float(Params['X']);
            
        if Params['Y'] is None:                                                 # If it exists, convert the parameter to a number
            YValue = None;
        else:
            YValue = float(Params['Y']);
            
        if Params['Z'] is None:                                                 # If it exists, convert the parameter to a number
            ZValue = None;
        else:
            ZValue = float(Params['Z']);
            
        if Params['I'] is None:                                                 # If it exists, convert the parameter to a number
            IValue = None;
        else:
            IValue = float(Params['I']);
            
        if Params['J'] is None:                                                 # If it exists, convert the parameter to a number
            JValue = None;
        else:
            JValue = float(Params['J']);
            
        if Params['K'] is None:                                                 # If it exists, convert the parameter to a number
            KValue = None;
        else:
            KValue = float(Params['K']);
    except:
        logging.error("A numeric conversion failed" + CommonErrorString);
        sys.exit(1);        

    Values = dict({'G': GValue, 'M': MValue, 'X': XValue, 'Y': YValue, 'Z': ZValue, 'I': IValue, 'J': JValue, 'K': KValue});

    return(Values);
    
# ******************************************************************************
# ParseLine() - parse a single line, return an adjusted arc or the original line
#
# I'm well aware exiting like this on any error without closing stuff is not
# exactly nice but I'm learning all of this right now, and I have a half-done
# PCB I can't engrave waiting (hopefully still aligned) on the mill table, ok?
# ******************************************************************************

def ParseLine(CurrentLine):
    
    global TextLinesHandled;                                                    # Stuff to be updated from this function needs to be declared
    global TextLinesIgnored;
    global PathLinesHandled;
    global PathLinesDropped;
    global PathArcsAdjusted;
    global UnitsMode;
    global CoordsMode;
    global WorkPlane;
    global CurrentPosition;

    TextLinesHandled += 1;                                                      # Mark processing another line of text
    IgnoringThisLine = True;                                                    # Assume it will be ignored unless found otherwise
    
    GParams = ScanForParams(CurrentLine);                                       # Retrieve any relevant G-words from the line (values only, as string)
    GValues = WordsToValues(GParams);                                           # Some parameters are more useful as numbers, some as strings: converting
    
    # *** OK, really starting to cut corners here. This should be rather more generalized. Needs rewriting AFTER that PCB is done.
    # *** For now, I'm assuming arc start point is never "unset" / imperial, absolute mode in plane XY, full stop. Sorry.
        
    if GValues['G'] == 0 or GValues['G'] == 1:                                  # Handle G0/G1 lines (does nothing as of 0.1)
        PathLinesHandled += 1;
        IgnoringThisLine = False;
        
    if GValues['G'] == 2 or GValues['G'] == 3:                                  # Handle G2/G3 lines (the point of all this)
        CurrentLine = AdjustArc(CurrentPosition, GValues, GParams);             # Get a new line instead of the old one with a recalculated center
        PathArcsAdjusted += 1;
        IgnoringThisLine = False;
        
    if GValues['X'] is not None:                                                # Harvest any new X-coordinate to update the current position
        CurrentPosition['X'] = GValues['X'];
        IgnoringThisLine = False;
        
    if GValues['Y'] is not None:                                                # Harvest any new Y-coordinate to update the current position
        CurrentPosition['Y'] = GValues['Y'];
        IgnoringThisLine = False;

    if GValues['Z'] is not None:                                                # Harvest any new Z-coordinate to update the current position
        CurrentPosition['Z'] = GValues['Z'];
        IgnoringThisLine = False;
            
    if IgnoringThisLine:                                                        # Count the line as ignored if nothing above triggered
        TextLinesIgnored += 1;
        
    # *** Done cutting corners. Seriously, this is supposed to be way more elaborate and discerning...    
        
    return(CurrentLine);                                                        # Return an output line for every input line of g-code

# ******************************************************************************
# Main() - fetch a file line by line and feed it to the parser / arc adjuster
#
# Note: modify the first line to change the amount of logged info if you wish.
# Valid values are DEBUG, INFO, WARNING, ERROR, CRITICAL.
# ******************************************************************************

def main():

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.WARNING)

    if len(sys.argv) < 2:                                                       # If there are no arguments, display version and usage info then exit
        print;
        print u"Line Bender {0} (C) 2012 Asztalos Attila Oszkár".format(VERSION);
        print u"Adjusts imprecise arcs in Line Grinder generated g-code";
        print u"Usage: linebend <input file> [<output file>]";
        sys.exit(1);
    else:                                                                       # Otherwise, fetch the input file name / path
        InFileName = sys.argv[1];
        
        if not os.path.isfile(InFileName):
            logging.error("File \"{0}\" does not seem to exist, exiting.".format(InFileName));
            sys.exit(1);
        
        if len(sys.argv) < 3:                                                   # If there is no second argument, construct an output file name / path
            (root, ext) = os.path.splitext(InFileName);                         # Split the input filename into path plus filename and extension
            OutFileName = root + "_BENT" + ext;                                 # Recombine them into a longer output filename
        else:
            OutFileName = sys.argv[2];                                          # Otherwise, fetch the output file name / path
    
    try:                                                                        # Attempt to open input file for reading
        InFile = open(InFileName, 'r');
    except:                                                                     # Exit if failed
        logging.error("Cannot open input file \"{0}\", exiting.".format(InFileName));
        sys.exit(1);

    logging.debug("Opened input file {0}.".format(InFileName));
    
    try:                                                                        # Attempt to open output file for writing
        OutFile = open(OutFileName, 'w');
    except:                                                                     # Exit if failed
        logging.error("Cannot open output file \"{0}\", exiting.".format(OutFileName));
        sys.exit(1);

    logging.debug("Opened output file {0}.".format(OutFileName));

    for CurrentInputLine in InFile:                                             # Traverse the file line by line looking for arcs to recalculate
        CurrentOutputLine = ParseLine(CurrentInputLine);        
        OutFile.write(CurrentOutputLine);
            
    try:                                                                        # Attempt to close output file
        OutFile.close();
    except:                                                                     # Exit if failed
        logging.error("Failed to close output file \"{0}\", exiting.".format(OutFileName));
        sys.exit(1);

    logging.debug("Closed output file {0}.".format(OutFileName));

    try:                                                                        # Attempt to close input file
        InFile.close();
    except:                                                                     # Exit if failed
        logging.error("Failed to close input file \"{0}\", exiting.".format(InFileName));
        sys.exit(1);

    logging.debug("Closed input file {0}.".format(InFileName));

    print;
    print u"Text lines handled: {0:>8}".format(TextLinesHandled);                # Display some processing stats
    print u"Text lines ignored: {0:>8}".format(TextLinesIgnored);
    
    print u"Path lines handled: {0:>8}".format(PathLinesHandled);
    print u"Path lines dropped: {0:>8}".format(PathLinesDropped);
    print u"Path arcs adjusted: {0:>8}".format(PathArcsAdjusted);
    print u"Path arcs unedited: {0:>8}".format(PathArcsUnedited);
    print;
    
    if PathArcsUnedited > 0:
        logging.warning("Full circle arcs were found and skipped ({0}).".format(PathArcsUnedited));
        
main();