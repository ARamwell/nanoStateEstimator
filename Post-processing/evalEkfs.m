
%ekfFileList = selector_multiFile();
%ekfFileList={"C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\stationary1\ekfResult_test.mat", "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\moving3_noTakeoff\ekfResult_test.mat", "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\moving2\ekfResult_test.mat"};
ekfFileList = {"C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\stationary1\ekfResult_testVel.mat"};

 %%
analyseEkf(16,"16el_rect_a0_", ekfFileList, "move2", false)