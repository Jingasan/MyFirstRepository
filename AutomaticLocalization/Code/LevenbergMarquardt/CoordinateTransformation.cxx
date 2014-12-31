/***** 点群の座標変換 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 点群の重心を原点座標に設定 ***/
int CoordinateTransformation(
	struct PointCloudInfo* modelInformation,					// モデル点群情報
	struct PointCloudInfo* targetInformation,					// 探索対象点群情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,				// 座標変換前のモデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,				// 座標変換前の探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud,	// 座標変換後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud	// 座標変換後の探索対象点群
){

	/* 変数の定義 */
	pcl::PointXYZRGB temp;	// 点座標の一時格納用変数

	/* 初期化 */
	modelInformation->Xmin = DBL_MAX;
	modelInformation->Ymin = DBL_MAX;
	modelInformation->Zmin = DBL_MAX;
	modelInformation->Xmax = -DBL_MAX;
	modelInformation->Ymax = -DBL_MAX;
	modelInformation->Zmax = -DBL_MAX;
	targetInformation->Xmin = DBL_MAX;
	targetInformation->Ymin = DBL_MAX;
	targetInformation->Zmin = DBL_MAX;
	targetInformation->Xmax = -DBL_MAX;
	targetInformation->Ymax = -DBL_MAX;
	targetInformation->Zmax = -DBL_MAX;

	/* モデル点群の端点の算出 */
	for( int i = 0; i < cloudM->points.size(); i++ ){
		
		/* X, Y, Z の最小値を設定 */
		if( cloudM->points[ i ].x < modelInformation->Xmin ) modelInformation->Xmin = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y < modelInformation->Ymin ) modelInformation->Ymin = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z < modelInformation->Zmin ) modelInformation->Zmin = cloudM->points[ i ].z;
		
		/* X, Y, Z の最大値を設定 */
		if( cloudM->points[ i ].x > modelInformation->Xmax ) modelInformation->Xmax = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y > modelInformation->Ymax ) modelInformation->Ymax = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z > modelInformation->Zmax ) modelInformation->Zmax = cloudM->points[ i ].z;
	}

	/* 探索対象点群の端点の算出 */
	for( int i = 0; i < cloudT->points.size(); i++ ){
		
		/* X, Y, Z の最小値を設定 */
		if( cloudT->points[ i ].x < targetInformation->Xmin ) targetInformation->Xmin = cloudT->points[ i ].x;
		if( cloudT->points[ i ].y < targetInformation->Ymin ) targetInformation->Ymin = cloudT->points[ i ].y;
		if( cloudT->points[ i ].z < targetInformation->Zmin ) targetInformation->Zmin = cloudT->points[ i ].z;
		
		/* X, Y, Z の最大値を設定 */
		if( cloudT->points[ i ].x > targetInformation->Xmax ) targetInformation->Xmax = cloudT->points[ i ].x;
		if( cloudT->points[ i ].y > targetInformation->Ymax ) targetInformation->Ymax = cloudT->points[ i ].y;
		if( cloudT->points[ i ].z > targetInformation->Zmax ) targetInformation->Zmax = cloudT->points[ i ].z;
	}
	
	/* 点群の重心をモデル情報として格納 */
	modelInformation->gravX = ( modelInformation->Xmax + modelInformation->Xmin ) / 2;
	modelInformation->gravY = ( modelInformation->Ymax + modelInformation->Ymin ) / 2;
	modelInformation->gravZ = ( modelInformation->Zmax + modelInformation->Zmin ) / 2;
	targetInformation->gravX = ( targetInformation->Xmax + targetInformation->Xmin ) / 2;
	targetInformation->gravY = ( targetInformation->Ymax + targetInformation->Ymin ) / 2;
	targetInformation->gravZ = ( targetInformation->Zmax + targetInformation->Zmin ) / 2;

	/* 点群の重心を原点に設定 */
	modelInformation->Xmin -= modelInformation->gravX; modelInformation->Xmax -= modelInformation->gravX;
	modelInformation->Ymin -= modelInformation->gravY; modelInformation->Ymax -= modelInformation->gravY;
	modelInformation->Zmin -= modelInformation->gravZ; modelInformation->Zmax -= modelInformation->gravZ;
	targetInformation->Xmin -= targetInformation->gravX; targetInformation->Xmax -= targetInformation->gravX;
	targetInformation->Ymin -= targetInformation->gravY; targetInformation->Ymax -= targetInformation->gravY;
	targetInformation->Zmin -= targetInformation->gravZ; targetInformation->Zmax -= targetInformation->gravZ;


	/* モデル点群 */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* モデル点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp.x = cloudM->points[ i ].x - modelInformation->gravX;
		temp.y = cloudM->points[ i ].y - modelInformation->gravY;
		temp.z = cloudM->points[ i ].z - modelInformation->gravZ;
		temp.r = 255;
		temp.g = 0;
		temp.b = 0;
		modelPointCloud->points.push_back( temp );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;

	/* 探索対象点群 */
	for( int i = 0; i < cloudT->size(); i++ ){
			
		/* 探索対象点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp.x = cloudT->points[ i ].x - targetInformation->gravX;
		temp.y = cloudT->points[ i ].y - targetInformation->gravY;
		temp.z = cloudT->points[ i ].z - targetInformation->gravZ;
		temp.r = 0;
		temp.g = 255;
		temp.b = 0;
		targetPointCloud->points.push_back( temp );
	}
	targetPointCloud->width = targetPointCloud->points.size();
	targetPointCloud->height = 1;



	return 0;
}