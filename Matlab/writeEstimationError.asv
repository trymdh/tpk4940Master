function writeEstimationError(estimationErrors)
%this function writes all the estimationErrors into .txt files that can be imported into Python

dlmwrite("transVecError.txt",estimationErrors.ExtrinsicsErrors.TranslationVectorsError,'delimiter','\t','newline','pc')
dlmwrite("rotVecError.txt",estimationErrors.ExtrinsicsErrors.RotationVectorsError,'delimiter','\t','newline','pc')

dlmwrite("FocalLengthError.txt",estimationErrors.IntrinsicsErrors.FocalLengthError,'delimiter','\t','newline','pc')
dlmwrite("skewError.txt",estimationErrors.IntrinsicsErrors.SkewError,'delimiter','\t','newline','pc')
dlmwrite("prinPointError.txt",estimationErrors.IntrinsicsErrors.PrincipalPointError,'delimiter','\t','newline','pc')
dlmwrite("radDistError.txt",estimationErrors.IntrinsicsErrors.RadialDistortionError,'delimiter','\t','newline','pc')
dlmwrite("tanDistError.txt",estimationErrors.IntrinsicsErrors.TangentialDistortionError,'delimiter','\t','newline','pc')

end