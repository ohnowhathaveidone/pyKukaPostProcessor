import postProc_kuka

testOut = postProc_kuka.SrcGenerator('test');
dummyPos = postProc_kuka.Position();
testOut.setBaseByPosition(dummyPos);
dummyPos = postProc_kuka.Position(x = 7, a = 37);
testOut.setBaseByPosition(dummyPos);
testOut.close();
