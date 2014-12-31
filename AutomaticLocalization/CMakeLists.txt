# �v������� CMake �̍Œ�o�[�W�����ԍ�
cmake_minimum_required( VERSION 2.8.9 )
if( COMMAND CMAKE_POLICY )
  cmake_policy( SET CMP0003 NEW )
endif()

# �v���W�F�N�g�f�B���N�g��( �\�����[�V�����t�@�C�� )���̎w��
project( AutomaticLocalization )


### CannyEdgeDetection ###

# �����������s�t�@�C����( �v���W�F�N�g )�Ƃ��̌��ƂȂ�\�[�X�v���O������
add_executable( CannyEdgeDetection
Code/CannyEdgeDetection/CannyEdgeDetection.cxx
Code/CannyEdgeDetection/PointCloudViewer.cxx
Code/CannyEdgeDetection/SavePointCloud.cxx
Code/CannyEdgeDetection/PointCloudReduction.cxx
Code/CannyEdgeDetection/FunctionDefinition.h
Code/CannyEdgeDetection/Configuration.h
)

# ITK �̃p�b�P�[�W�̌��o
find_package( ITK REQUIRED )
# �C���N���[�h�p�X�̎w��
include( ${ITK_USE_FILE} )
# �����N���郉�C�u�����̎w��
target_link_libraries( CannyEdgeDetection ${ITK_LIBRARIES} )

# PCL �̃p�b�P�[�W�̌��o
find_package( PCL 1.6 REQUIRED )
# �C���N���[�h�p�X�̎w��
include_directories( ${PCL_INCLUDE_DIRS} )
# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( CannyEdgeDetection ${PCL_LIBRARIES} )


### AutomaticLocalization ###

# �����������s�t�@�C����( �v���W�F�N�g )�Ƃ��̌��ƂȂ�\�[�X�v���O������
add_executable( AutomaticLocalization
Code/AutomaticLocalization/AutomaticLocalization.cxx
Code/AutomaticLocalization/BinarizeDistanceField.cxx
Code/AutomaticLocalization/CreateDistanceField.cxx
Code/AutomaticLocalization/ExhaustiveSearch.cxx
Code/AutomaticLocalization/MergedPointDataViewer.cxx
Code/AutomaticLocalization/PointCloudViewer.cxx
Code/AutomaticLocalization/SavePointCloud.cxx
Code/AutomaticLocalization/OptimizationWithICP.cxx
Code/AutomaticLocalization/OptimizationWithLevMar.cxx
Code/AutomaticLocalization/ConvertRodrigues.cxx
Code/AutomaticLocalization/PartialDerivativeForRodrigues.cxx
Code/AutomaticLocalization/FunctionDefinition.h
Code/AutomaticLocalization/Configuration.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( AutomaticLocalization ${PCL_LIBRARIES} )


## PointCloudReduction ##

# �����������s�t�@�C����( �v���W�F�N�g )�Ƃ��̌��ƂȂ�\�[�X�v���O������
add_executable( PointCloudReduction
Code/PointCloudReduction/PointCloudReduction.cxx
Code/PointCloudReduction/LoadPointCloud.cxx
Code/PointCloudReduction/PointCloudReductor.cxx
Code/PointCloudReduction/SavePointCloud.cxx
Code/PointCloudReduction/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( PointCloudReduction ${PCL_LIBRARIES} )


## ConvertPCDtoPLY ##

# �����������s�t�@�C����( �v���W�F�N�g )�Ƃ��̌��ƂȂ�\�[�X�v���O������
add_executable( ConvertPCDtoPLY
Code/ConvertPCDtoPLY/ConvertPCDtoPLY.cxx
Code/ConvertPCDtoPLY/LoadPointCloud.cxx
Code/ConvertPCDtoPLY/SavePointCloud.cxx
Code/ConvertPCDtoPLY/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( ConvertPCDtoPLY ${PCL_LIBRARIES} )


## UniformSamplingOnSO3 ##
add_executable( UniformSamplingOnSO3
Code/UniformSamplingOnSO3/UniformSamplingOnSO3.cxx
Code/UniformSamplingOnSO3/find_point.cxx
Code/UniformSamplingOnSO3/mk_pix2xy.cxx
Code/UniformSamplingOnSO3/pix2ang_nest.cxx
Code/UniformSamplingOnSO3/hopf2quat.cxx
Code/UniformSamplingOnSO3/so3_sequence.h
)


## UniformSamplingOnSO3Viewer ##
add_executable( UniformSamplingOnSO3Viewer
Code/UniformSamplingOnSO3Viewer/UniformSamplingOnSO3Viewer.cxx
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( UniformSamplingOnSO3Viewer ${PCL_LIBRARIES} )


## Generate2DSliceImage ##
add_executable( Generate2DSliceImage
Code/Generate2DSliceImage/Convert3DImageTo2DImages.cxx
)

# �����N���郉�C�u�����̎w��
target_link_libraries( Generate2DSliceImage ${ITK_LIBRARIES} )


## PrincipalComponentAnalysis ##
add_executable( PrincipalComponentAnalysis
Code/PrincipalComponentAnalysis/PrincipalComponentAnalysis.cxx
Code/PrincipalComponentAnalysis/PointCloudViewer.cxx
Code/PrincipalComponentAnalysis/SavePointCloud.cxx
Code/PrincipalComponentAnalysis/Configuration.h
Code/PrincipalComponentAnalysis/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( PrincipalComponentAnalysis ${PCL_LIBRARIES} )


## GenerateNoisyPointCloud ##
add_executable( GenerateNoisyPointCloud
Code/GenerateNoisyPointCloud/GenerateNoisyPointCloud.cxx
Code/GenerateNoisyPointCloud/PointCloudViewer.cxx
Code/GenerateNoisyPointCloud/SavePointCloud.cxx
Code/GenerateNoisyPointCloud/Configuration.h
Code/GenerateNoisyPointCloud/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( GenerateNoisyPointCloud ${PCL_LIBRARIES} )


## PlusGaussianNoiseToPointCloud ##
add_executable( PlusGaussianNoiseToPointCloud
Code/PlusGaussianNoiseToPointCloud/PlusGaussianNoise.cxx
Code/PlusGaussianNoiseToPointCloud/PointCloudViewer.cxx
Code/PlusGaussianNoiseToPointCloud/SavePointCloud.cxx
Code/PlusGaussianNoiseToPointCloud/Configuration.h
Code/PlusGaussianNoiseToPointCloud/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( PlusGaussianNoiseToPointCloud ${PCL_LIBRARIES} )


## GenerateMultiResolutionalVolumeData ##
add_executable( GenerateMultiResolutionalVolumeData
Code/GenerateMultiResolutionalVolumeData/GenerateMultiResolutionalVolumeData.cxx
Code/GenerateMultiResolutionalVolumeData/Configuration.h
)

# �����N���郉�C�u�����̎w��
target_link_libraries( GenerateMultiResolutionalVolumeData ${ITK_LIBRARIES} )

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( GenerateMultiResolutionalVolumeData ${PCL_LIBRARIES} )


## GenerateSurfaceMeshData ##
add_executable( GenerateSurfaceMeshData
Code/GenerateSurfaceMeshData/MarchingCubes.cxx
Code/GenerateSurfaceMeshData/Configuration.h
)

# �����N���郉�C�u�����̎w��
target_link_libraries( GenerateSurfaceMeshData ${ITK_LIBRARIES} )

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( GenerateSurfaceMeshData ${PCL_LIBRARIES} )


## CPDistanceVisualization ##
add_executable( CPDistanceVisualization
Code/CPDistanceVisualization/CreateHistogram.cxx
Code/CPDistanceVisualization/Configuration.h
)


## LevenbergMarquardt ##
add_executable( LevenbergMarquardt
Code/LevenbergMarquardt/LevenbergMarquardt.cxx
Code/LevenbergMarquardt/ConvertRodrigues.cxx
Code/LevenbergMarquardt/CoordinateTransformation.cxx
Code/LevenbergMarquardt/PartialDerivativeForRodrigues.cxx
Code/LevenbergMarquardt/ReconstructRFromRodrigues.cxx
Code/LevenbergMarquardt/Configuration.h
Code/LevenbergMarquardt/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( LevenbergMarquardt ${PCL_LIBRARIES} )


## TransformPointCloud ##
add_executable( TransformPointCloud
Code/TransformPointCloud/TransformPointCloud.cxx
Code/TransformPointCloud/PointCloudViewer.cxx
Code/TransformPointCloud/SavePointCloud.cxx
Code/TransformPointCloud/Configuration.h
Code/TransformPointCloud/FunctionDefinition.h
)

# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( TransformPointCloud ${PCL_LIBRARIES} )

