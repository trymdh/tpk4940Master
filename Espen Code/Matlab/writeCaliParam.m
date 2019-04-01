function writeCaliParam(cameraParams)
%this function writes all the cameraParameters into .txt
%files that can be imported into Python
dlmwrite('IntrinsicMatrix.txt',cameraParams.IntrinsicMatrix.','delimiter','\t','newline','pc')
dlmwrite('PrincipalPoint.txt',cameraParams.PrincipalPoint,'delimiter','\t','newline','pc')
dlmwrite('RadialDistortion.txt',cameraParams.RadialDistortion,'delimiter','\t','newline','pc')
dlmwrite('TangentialDistortion.txt',cameraParams.TangentialDistortion,'delimiter','\t','newline','pc')


%to create a .txt file containing all the relative rotations between the 
%scene and the camera we need to do the following:
rM = cameraParams.RotationMatrices;
for i = 1:size(rM,3)
    filename = 'RotationMatrices.txt';
    data = rM(:,:,i);
    dlmwrite(filename,data,'-append','delimiter','\t','newline','pc')
end

dlmwrite('MeanReprojectionError.txt',cameraParams.MeanReprojectionError,'delimiter','\t','newline','pc')
dlmwrite('TranslationVectors.txt',cameraParams.TranslationVectors,'delimiter','\t','newline','pc')
dlmwrite('ReprojectionErrors.txt',cameraParams.ReprojectionErrors,'delimiter','\t','newline','pc')
end