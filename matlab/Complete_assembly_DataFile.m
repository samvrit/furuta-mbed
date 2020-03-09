% Simscape(TM) Multibody(TM) version: 6.0

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(40).translation = [0.0 0.0 0.0];
smiData.RigidTransform(40).angle = 0.0;
smiData.RigidTransform(40).axis = [0.0 0.0 0.0];
smiData.RigidTransform(40).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [68.635485579652581 -12.396384198010347 83.582566177606765];  % mm
smiData.RigidTransform(1).angle = 3.1415869809834547;  % rad
smiData.RigidTransform(1).axis = [-1 -1.5127786666358372e-20 5.3336282347687803e-15];
smiData.RigidTransform(1).ID = 'B[Assembly_base-1:-:Assembly_link1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-2.8351391884699532e-13 -5.7641716920409092e-14 -111];  % mm
smiData.RigidTransform(2).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(2).axis = [-1 -1.4786998643569447e-36 1.7457406694158206e-15];
smiData.RigidTransform(2).ID = 'F[Assembly_base-1:-:Assembly_link1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-9.7488683792334996e-13 299.99999999999989 0];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931975;  % rad
smiData.RigidTransform(3).axis = [0.57735026918962651 -0.5773502691896244 0.57735026918962651];
smiData.RigidTransform(3).ID = 'B[Assembly_link1-1:-:Assembly_link2-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-1.8616219676914625e-12 -1.1285761762829766e-18 -1.9895196600962707e-13];  % mm
smiData.RigidTransform(4).angle = 2.0943951023931979;  % rad
smiData.RigidTransform(4).axis = [0.57735026918962662 -0.57735026918962662 0.57735026918962407];
smiData.RigidTransform(4).ID = 'F[Assembly_link1-1:-:Assembly_link2-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-9.0205620750793969e-13 24.924999999999031 300];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931935;  % rad
smiData.RigidTransform(5).axis = [-0.57735026918962506 -0.57735026918962506 -0.57735026918962717];
smiData.RigidTransform(5).ID = 'B[Assembly_link2-2:-:Assembly_link3-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [1.2724840193093443e-12 -5.0000000000030251 1.1201797376779368e-12];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931935;  % rad
smiData.RigidTransform(6).axis = [-0.57735026918962518 -0.57735026918962329 -0.57735026918962895];
smiData.RigidTransform(6).ID = 'F[Assembly_link2-2:-:Assembly_link3-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-9.0205620750793969e-13 5.7249999999990369 299.99999999999989];  % mm
smiData.RigidTransform(7).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(7).axis = [1 0 0];
smiData.RigidTransform(7).ID = 'AssemblyGround[Assembly_link2-2:bearing-6mm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-9.0205620750793969e-13 29.924999999999034 300];  % mm
smiData.RigidTransform(8).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(8).axis = [1 0 0];
smiData.RigidTransform(8).ID = 'AssemblyGround[Assembly_link2-2:bearing-6mm-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [9.5300203199993181 -11.275000000003088 300.00002031999975];  % mm
smiData.RigidTransform(9).angle = 3.1415926535897882;  % rad
smiData.RigidTransform(9).axis = [-2.4688501310780157e-15 -0.7071067811865438 -0.70710678118655124];
smiData.RigidTransform(9).ID = 'AssemblyGround[Assembly_link2-2:position-sensor-pcb-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [-1.9567680809018384e-12 15.325000000000033 0];  % mm
smiData.RigidTransform(10).angle = 3.1415926535897865;  % rad
smiData.RigidTransform(10).axis = [-0 -1 -0];
smiData.RigidTransform(10).ID = 'AssemblyGround[Assembly_link2-2:link-2-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [-9.4368957093138306e-13 15.325000000000033 299.99999999999989];  % mm
smiData.RigidTransform(11).angle = 2.0943951023931993;  % rad
smiData.RigidTransform(11).axis = [0.57735026918962518 0.57735026918962695 0.57735026918962518];
smiData.RigidTransform(11).ID = 'AssemblyGround[Assembly_link2-2:bearing-mount-1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [-9.5300000000008502 -11.275000000000091 299.99999999999994];  % mm
smiData.RigidTransform(12).angle = 1.5707963267948999;  % rad
smiData.RigidTransform(12).axis = [0 0 1];
smiData.RigidTransform(12).ID = 'AssemblyGround[Assembly_link2-2:pcb-standoffs-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [9.5299999999991503 -11.274999999999979 299.99999999999983];  % mm
smiData.RigidTransform(13).angle = 1.5707963267948999;  % rad
smiData.RigidTransform(13).axis = [0 0 1];
smiData.RigidTransform(13).ID = 'AssemblyGround[Assembly_link2-2:pcb-standoffs-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [-1.8596235662471372e-12 -14.749999999999986 0];  % mm
smiData.RigidTransform(14).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(14).axis = [1 0 0];
smiData.RigidTransform(14).ID = 'AssemblyGround[Assembly_link2-2:pin-encoder-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [-1.8735013540549517e-12 -6.3500000000000227 0];  % mm
smiData.RigidTransform(15).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(15).axis = [1 0 0];
smiData.RigidTransform(15).ID = 'AssemblyGround[Assembly_link2-2:coupling-6mm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [-1.8735013540549517e-12 2.9500000000000082 0];  % mm
smiData.RigidTransform(16).angle = 1.5707963267948861;  % rad
smiData.RigidTransform(16).axis = [-1 0 0];
smiData.RigidTransform(16).ID = 'AssemblyGround[Assembly_link2-2:hub-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [-1.8735013540549517e-12 12.149999999999995 0];  % mm
smiData.RigidTransform(17).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(17).axis = [1 0 0];
smiData.RigidTransform(17).ID = 'AssemblyGround[Assembly_link2-2:pin-encoder-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [69.42427718113133 -12.403822103818618 120.46392762221913];  % mm
smiData.RigidTransform(18).angle = 1.5674063189740435;  % rad
smiData.RigidTransform(18).axis = [1 5.2313041069630832e-15 -5.2313039110716018e-15];
smiData.RigidTransform(18).ID = 'AssemblyGround[Assembly_base-1:bearing-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [19.424277181130154 10.239547510084323 14.570313894557819];  % mm
smiData.RigidTransform(19).angle = 2.0943967399343539;  % rad
smiData.RigidTransform(19).axis = [0.57735081503564856 -0.57734917749602832 -0.57735081503565244];
smiData.RigidTransform(19).ID = 'AssemblyGround[Assembly_base-1:mount-sides-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [69.424277181131373 -12.397042101163068 122.46391613007711];  % mm
smiData.RigidTransform(20).angle = 2.0924389875485199;  % rad
smiData.RigidTransform(20).axis = [0.57669675521419872 0.57865508297329749 0.57669675521419472];
smiData.RigidTransform(20).ID = 'AssemblyGround[Assembly_base-1:mount-top-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [68.63548557965234 -12.396270745883578 63.582566177928555];  % mm
smiData.RigidTransform(21).angle = 1.5708019994012354;  % rad
smiData.RigidTransform(21).axis = [1 6.8773064180519163e-15 -3.7899110392510388e-15];
smiData.RigidTransform(21).ID = 'AssemblyGround[Assembly_base-1:mount-motor-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [118.63548557965196 -35.031671959790785 14.570022019590814];  % mm
smiData.RigidTransform(22).angle = 2.0943983774770967;  % rad
smiData.RigidTransform(22).axis = [0.57735136088012895 0.57734808580243091 0.57735136088012484];
smiData.RigidTransform(22).ID = 'AssemblyGround[Assembly_base-1:mount-sides-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [69.424277181131416 -12.380092094524191 127.46388739972211];  % mm
smiData.RigidTransform(23).angle = 2.0924389875485199;  % rad
smiData.RigidTransform(23).axis = [0.57669675521419872 0.57865508297329749 0.57669675521419472];
smiData.RigidTransform(23).ID = 'AssemblyGround[Assembly_base-1:encoder-mount-1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(24).translation = [68.6354855796517 -12.395919044290579 1.5825661789260781];  % mm
smiData.RigidTransform(24).angle = 5.6726063386656777e-06;  % rad
smiData.RigidTransform(24).axis = [1 1.8804859411597987e-09 0];
smiData.RigidTransform(24).ID = 'AssemblyGround[Assembly_base-1:motor-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(25).translation = [-9.5300000000008289 258.79999999999978 0];  % mm
smiData.RigidTransform(25).angle = 1.5707963267948999;  % rad
smiData.RigidTransform(25).axis = [0 0 1];
smiData.RigidTransform(25).ID = 'AssemblyGround[Assembly_link1-1:pcb-standoffs-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(26).translation = [9.5299999999991645 258.79999999999984 0];  % mm
smiData.RigidTransform(26).angle = 1.5707963267948999;  % rad
smiData.RigidTransform(26).axis = [0 0 1];
smiData.RigidTransform(26).ID = 'AssemblyGround[Assembly_link1-1:pcb-standoffs-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(27).translation = [-8.7430063189231078e-13 270.7999999999999 0];  % mm
smiData.RigidTransform(27).angle = 2.0943951023931975;  % rad
smiData.RigidTransform(27).axis = [-0.57735026918962651 0.5773502691896244 0.57735026918962651];
smiData.RigidTransform(27).ID = 'AssemblyGround[Assembly_link1-1:bearing-6mm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(28).translation = [-9.5756735873919752e-13 294.99999999999989 0];  % mm
smiData.RigidTransform(28).angle = 2.0943951023931975;  % rad
smiData.RigidTransform(28).axis = [-0.57735026918962651 0.5773502691896244 0.57735026918962651];
smiData.RigidTransform(28).ID = 'AssemblyGround[Assembly_link1-1:bearing-6mm-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(29).translation = [0 0 -97.999999999999986];  % mm
smiData.RigidTransform(29).angle = 3.1415926535897909;  % rad
smiData.RigidTransform(29).axis = [-0.70710678118654624 -0.70710678118654879 1.2344250655411311e-15];
smiData.RigidTransform(29).ID = 'AssemblyGround[Assembly_link1-1:coupling-6mm-8mm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(30).translation = [-9.298117831235686e-13 285.39999999999981 0];  % mm
smiData.RigidTransform(30).angle = 2.0943951023931993;  % rad
smiData.RigidTransform(30).axis = [0.57735026918962518 0.57735026918962695 0.57735026918962518];
smiData.RigidTransform(30).ID = 'AssemblyGround[Assembly_link1-1:bearing-mount-1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(31).translation = [0 0 -31];  % mm
smiData.RigidTransform(31).angle = 2.0943951023931993;  % rad
smiData.RigidTransform(31).axis = [0.57735026918962518 0.57735026918962695 0.57735026918962518];
smiData.RigidTransform(31).ID = 'AssemblyGround[Assembly_link1-1:link-1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(32).translation = [0 0 -49.999999999999986];  % mm
smiData.RigidTransform(32).angle = 2.0943951023931993;  % rad
smiData.RigidTransform(32).axis = [0.57735026918962518 0.57735026918962695 0.57735026918962518];
smiData.RigidTransform(32).ID = 'AssemblyGround[Assembly_link1-1:shaft-collar-8mm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(33).translation = [0 0 -109.99999999999999];  % mm
smiData.RigidTransform(33).angle = 2.0943951023931993;  % rad
smiData.RigidTransform(33).axis = [0.57735026918962518 0.57735026918962695 0.57735026918962518];
smiData.RigidTransform(33).ID = 'AssemblyGround[Assembly_link1-1:shaft-1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(34).translation = [9.5300203199991387 258.79999999999978 2.0319999988194226e-05];  % mm
smiData.RigidTransform(34).angle = 3.1415926535897882;  % rad
smiData.RigidTransform(34).axis = [2.7714460939274379e-21 -0.70710678118654746 -0.70710678118654757];
smiData.RigidTransform(34).ID = 'AssemblyGround[Assembly_link1-1:position-sensor-pcb-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(35).translation = [0 18.500000000000018 0];  % mm
smiData.RigidTransform(35).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(35).axis = [1 0 0];
smiData.RigidTransform(35).ID = 'AssemblyGround[Assembly_link3-1:link-3-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(36).translation = [0 2.9500000000000082 0];  % mm
smiData.RigidTransform(36).angle = 1.5707963267948861;  % rad
smiData.RigidTransform(36).axis = [-1 0 0];
smiData.RigidTransform(36).ID = 'AssemblyGround[Assembly_link3-1:hub-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(37).translation = [0 -14.749999999999986 0];  % mm
smiData.RigidTransform(37).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(37).axis = [1 0 0];
smiData.RigidTransform(37).ID = 'AssemblyGround[Assembly_link3-1:pin-encoder-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(38).translation = [0 12.149999999999995 0];  % mm
smiData.RigidTransform(38).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(38).axis = [1 0 0];
smiData.RigidTransform(38).ID = 'AssemblyGround[Assembly_link3-1:pin-encoder-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(39).translation = [0 -6.3500000000000227 0];  % mm
smiData.RigidTransform(39).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(39).axis = [1 0 0];
smiData.RigidTransform(39).ID = 'AssemblyGround[Assembly_link3-1:coupling-6mm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(40).translation = [0 0 0];  % mm
smiData.RigidTransform(40).angle = 0;  % rad
smiData.RigidTransform(40).axis = [0 0 0];
smiData.RigidTransform(40).ID = 'RootGround[Assembly_base-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(19).mass = 0.0;
smiData.Solid(19).CoM = [0.0 0.0 0.0];
smiData.Solid(19).MoI = [0.0 0.0 0.0];
smiData.Solid(19).PoI = [0.0 0.0 0.0];
smiData.Solid(19).color = [0.0 0.0 0.0];
smiData.Solid(19).opacity = 0.0;
smiData.Solid(19).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.0042001740477044767;  % kg
smiData.Solid(1).CoM = [0 0 2.5000000000000004];  % mm
smiData.Solid(1).MoI = [0.064233667645525516 0.064233667645525502 0.11096661009228238];  % kg*mm^2
smiData.Solid(1).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(1).color = [0.89803921568627454 0.89803921568627454 0.89803921568627454];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'bearing-6mm*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.0024743905653210265;  % kg
smiData.Solid(2).CoM = [9.5543112087728357 -9.6333089228937308 -0.59818336989241039];  % mm
smiData.Solid(2).MoI = [0.2861303256487488 0.15305002110833971 0.43053313408038718];  % kg*mm^2
smiData.Solid(2).PoI = [-0.0030671505259169613 0.00043921707776229491 0.00041684440454895958];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'position-sensor-pcb*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.04921136784927059;  % kg
smiData.Solid(3).CoM = [-1.5875176394885963e-06 -3.1038552918482052e-07 -132.83239490295514];  % mm
smiData.Solid(3).MoI = [340.20853585051134 340.50048087386438 0.62380019809597353];  % kg*mm^2
smiData.Solid(3).PoI = [1.6296635713978039e-06 8.2466417109998833e-06 -4.3500657322461036e-08];  % kg*mm^2
smiData.Solid(3).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'link-2*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.037156508519097589;  % kg
smiData.Solid(4).CoM = [0.051755731072656776 -11.795027714745041 3.1428178947330095e-07];  % mm
smiData.Solid(4).MoI = [7.5337743433414071 6.166463377818487 9.4168754823947847];  % kg*mm^2
smiData.Solid(4).PoI = [-1.8131497132975705e-07 7.5692119514714571e-09 -0.022682568740502337];  % kg*mm^2
smiData.Solid(4).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = 'bearing-mount-1*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.0018142697574481056;  % kg
smiData.Solid(5).CoM = [6.0000000000000009 0 0];  % mm
smiData.Solid(5).MoI = [0.005669592992025331 0.024606033585389934 0.024606033585389934];  % kg*mm^2
smiData.Solid(5).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(5).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = 'pcb-standoffs*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.0019848582385380317;  % kg
smiData.Solid(6).CoM = [0 0 13];  % mm
smiData.Solid(6).MoI = [0.1162796118076863 0.1162796118076863 0.0089318620734211464];  % kg*mm^2
smiData.Solid(6).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(6).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = 'pin-encoder*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.0064412466875633225;  % kg
smiData.Solid(7).CoM = [0 0 7.9500000000000002];  % mm
smiData.Solid(7).MoI = [0.2383647749199683 0.2383647749199683 0.20821329917548442];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(7).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = 'coupling-6mm*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(8).mass = 0.0070390367916700197;  % kg
smiData.Solid(8).CoM = [0 6.5695749304146847e-05 6.0714044706874741];  % mm
smiData.Solid(8).MoI = [0.30000728337099924 0.29999849938121143 0.5367117818864946];  % kg*mm^2
smiData.Solid(8).PoI = [-2.9068444562028465e-07 0 0];  % kg*mm^2
smiData.Solid(8).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(8).opacity = 1;
smiData.Solid(8).ID = 'hub*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(9).mass = 0.0023090706003884979;  % kg
smiData.Solid(9).CoM = [0 3.5 0];  % mm
smiData.Solid(9).MoI = [0.088514373014892372 0.15817133612661208 0.088514373014892414];  % kg*mm^2
smiData.Solid(9).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(9).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(9).opacity = 1;
smiData.Solid(9).ID = 'bearing*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(10).mass = 0.76663696850134366;  % kg
smiData.Solid(10).CoM = [22.635679242411822 -37.090126225257713 4.7625000000000011];  % mm
smiData.Solid(10).MoI = [5717.4278629261116 642.73318988247058 6348.5687831226833];  % kg*mm^2
smiData.Solid(10).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(10).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(10).opacity = 1;
smiData.Solid(10).ID = 'mount-sides*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(11).mass = 0.25948003319176943;  % kg
smiData.Solid(11).CoM = [0 -0.025329378298368298 0];  % mm
smiData.Solid(11).MoI = [223.6364261644346 444.24801662785109 224.97485092523675];  % kg*mm^2
smiData.Solid(11).PoI = [0 0 0.0064804730265858694];  % kg*mm^2
smiData.Solid(11).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(11).opacity = 1;
smiData.Solid(11).ID = 'mount-top*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(12).mass = 0.22342550520847593;  % kg
smiData.Solid(12).CoM = [0 0.52275366621635633 0.034182538845153904];  % mm
smiData.Solid(12).MoI = [207.25843212082088 410.02797037741067 206.08233204430971];  % kg*mm^2
smiData.Solid(12).PoI = [-0.0074634755504738208 0 0];  % kg*mm^2
smiData.Solid(12).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(12).opacity = 1;
smiData.Solid(12).ID = 'mount-motor*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(13).mass = 0.0055873811456433079;  % kg
smiData.Solid(13).CoM = [0 2.5925173401741342 0];  % mm
smiData.Solid(13).MoI = [1.1021646234887426 1.3988100450590841 0.33238376776973799];  % kg*mm^2
smiData.Solid(13).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(13).color = [1 1 1];
smiData.Solid(13).opacity = 1;
smiData.Solid(13).ID = 'encoder-mount-1*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(14).mass = 0.058348800355123243;  % kg
smiData.Solid(14).CoM = [0 0 30.43708609271523];  % mm
smiData.Solid(14).MoI = [22.798717500119213 22.798717500119213 8.8422100511665764];  % kg*mm^2
smiData.Solid(14).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(14).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(14).opacity = 1;
smiData.Solid(14).ID = 'motor*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(15).mass = 0.018661060362323376;  % kg
smiData.Solid(15).CoM = [0 0 12.748181818181818];  % mm
smiData.Solid(15).MoI = [1.4868354735622944 1.4868354735622944 1.0431363096535475];  % kg*mm^2
smiData.Solid(15).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(15).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(15).opacity = 1;
smiData.Solid(15).ID = 'coupling-6mm-8mm*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(16).mass = 0.16298677130509001;  % kg
smiData.Solid(16).CoM = [141.76425715370939 7.9029375265137985e-08 -3.5388266138146433e-08];  % mm
smiData.Solid(16).MoI = [8.8013446003434606 1365.3440675932954 1358.2847260169917];  % kg*mm^2
smiData.Solid(16).PoI = [-1.1743979961175819e-09 -6.3907267459703988e-07 1.439204332253259e-06];  % kg*mm^2
smiData.Solid(16).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(16).opacity = 1;
smiData.Solid(16).ID = 'link-1*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(17).mass = 0.0091361441357208154;  % kg
smiData.Solid(17).CoM = [0 7.4999999999999991 0];  % mm
smiData.Solid(17).MoI = [0.76335096316433027 1.1128216379108118 0.69207607983601205];  % kg*mm^2
smiData.Solid(17).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(17).color = [0.25098039215686274 0.25098039215686274 0.25098039215686274];
smiData.Solid(17).opacity = 1;
smiData.Solid(17).ID = 'shaft-collar-8mm*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(18).mass = 0.010178760197630929;  % kg
smiData.Solid(18).CoM = [0 37.5 0];  % mm
smiData.Solid(18).MoI = [4.8120088834300221 0.08143008158104742 4.8120088834300221];  % kg*mm^2
smiData.Solid(18).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(18).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(18).opacity = 1;
smiData.Solid(18).ID = 'shaft-1*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(19).mass = 0.078942324918215762;  % kg
smiData.Solid(19).CoM = [2.9200830014125403e-07 219.65434062244054 3.175000214230717];  % mm
smiData.Solid(19).MoI = [1399.41731859059 0.92768616098855416 1399.8141536338494];  % kg*mm^2
smiData.Solid(19).PoI = [3.0676129778317898e-06 4.6748386730856476e-08 3.6467264823093668e-06];  % kg*mm^2
smiData.Solid(19).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(19).opacity = 1;
smiData.Solid(19).ID = 'link-3*:*Default';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(3).Rz.Pos = 0.0;
smiData.RevoluteJoint(3).ID = '';

smiData.RevoluteJoint(1).Rz.Pos = 1.1661223074350711e-18;  % deg
smiData.RevoluteJoint(1).ID = '[Assembly_base-1:-:Assembly_link1-1]';

smiData.RevoluteJoint(2).Rz.Pos = 3.9694545696077346e-13;  % deg
smiData.RevoluteJoint(2).ID = '[Assembly_link1-1:-:Assembly_link2-2]';

smiData.RevoluteJoint(3).Rz.Pos = 4.5427407241477197e-14;  % deg
smiData.RevoluteJoint(3).ID = '[Assembly_link2-2:-:Assembly_link3-1]';

