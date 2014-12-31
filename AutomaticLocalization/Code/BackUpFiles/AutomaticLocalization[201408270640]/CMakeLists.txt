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
Code/AutomaticLocalization/FunctionDefinition.h
Code/AutomaticLocalization/Configuration.h
)

# PCL �̃p�b�P�[�W�̌��o
find_package( PCL 1.6 REQUIRED )
# �C���N���[�h�p�X�̎w��
include_directories( ${PCL_INCLUDE_DIRS} )
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

# PCL �̃p�b�P�[�W�̌��o
find_package( PCL 1.6 REQUIRED )
# �C���N���[�h�p�X�̎w��
include_directories( ${PCL_INCLUDE_DIRS} )
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

# PCL �̃p�b�P�[�W�̌��o
find_package( PCL 1.6 REQUIRED )
# �C���N���[�h�p�X�̎w��
include_directories( ${PCL_INCLUDE_DIRS} )
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

# PCL �̃p�b�P�[�W�̌��o
find_package( PCL 1.6 REQUIRED )
# �C���N���[�h�p�X�̎w��
include_directories( ${PCL_INCLUDE_DIRS} )
# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( UniformSamplingOnSO3Viewer ${PCL_LIBRARIES} )


## Generate2DSliceImage ##
add_executable( Generate2DSliceImage
Code/Generate2DSliceImage/Convert3DImageTo2DImages.cxx
)

# ITK �̃p�b�P�[�W�̌��o
find_package( ITK REQUIRED )
# �C���N���[�h�p�X�̎w��
include( ${ITK_USE_FILE} )
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

# PCL �̃p�b�P�[�W�̌��o
find_package( PCL 1.6 REQUIRED )
# �C���N���[�h�p�X�̎w��
include_directories( ${PCL_INCLUDE_DIRS} )
# �����N���郉�C�u�����̎w��
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( PrincipalComponentAnalysis ${PCL_LIBRARIES} )