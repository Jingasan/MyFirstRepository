/***** 探索対象点群とモデル点群との距離評価 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "createDistanceField.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 探索対象点群とモデル点群との距離評価 ***/
void EvaluateDistance(
	int** DistanceField,						// DF
	struct ModelInformation* modelinformation,	// 探索対象点群の各種情報
	int* SumofDistance,							// 探索対象点群とモデル点群との距離評価値
	int* minSumofDistance,						// 探索対象点群とモデル点群との距離評価値の最小値
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud	// モデル点群
){
	
	/* 変数の定義 */
	int iSize = modelinformation->iSize;			// DF の i 方向のサイズ
	int jSize = modelinformation->jSize;			// DF の j 方向のサイズ
	int kSize = modelinformation->kSize;			// DF の k 方向のサイズ
	int ijSize = iSize * jSize;						// DF の i, j 方向の面積
	double targetXSize = modelinformation->deltaX;	// 探索対象点群の X 方向の最大幅 
	double targetYSize = modelinformation->deltaY;	// 探索対象点群の Y 方向の最大幅
	double targetZSize = modelinformation->deltaZ;	// 探索対象点群の Z 方向の最大幅
	double gravX = modelinformation->gravX;	// 探索対象の重心 X
	double gravY = modelinformation->gravY;	// 探索対象の重心 Y
	double gravZ = modelinformation->gravZ;	// 探索対象の重心 Z
	int i, j, k;	// モデル点群を DF 座標に直した値
	int sum = 0;	// 探索対象点群とモデル点群との距離評価値の合計

	
	/*** 探索対象点群とモデル点群との距離評価値の算出 ***/
	for( int n = 0; n < cloud->size(); n++ ){

		/* モデル点群を DF 内にある探索対象点群の座標系へ移動 */
		cloud->points[ n ].x = cloud->points[ n ].x * TOMETER - gravX;
		cloud->points[ n ].y = cloud->points[ n ].y * TOMETER - gravY;
		cloud->points[ n ].z = cloud->points[ n ].z * TOMETER - gravZ;

		/* モデル点群の DF 上での座標値の算出 */
		i = (int)( ( (double)iSize * ( cloud->points[ n ].x + targetXSize ) / ( 2.0 * targetXSize ) ) );//+ 0.5 );
		j = (int)( ( (double)jSize * ( cloud->points[ n ].y + targetYSize ) / ( 2.0 * targetYSize ) ) );//+ 0.5 );
		k = (int)( ( (double)kSize * ( cloud->points[ n ].z + targetZSize ) / ( 2.0 * targetZSize ) ) );//+ 0.5 );

#ifdef CUT
		i = (int)( (double)( ( (double)iSize * cloud->points[ n ].x ) / ( 2.0 * targetXSize ) ) + ( (double)iSize / 2.0 ) + 0.5 );
		j = (int)( (double)( ( (double)jSize * cloud->points[ n ].y ) / ( 2.0 * targetYSize ) ) + ( (double)jSize / 2.0 ) + 0.5 );
		k = (int)( (double)( ( (double)kSize * cloud->points[ n ].z ) / ( 2.0 * targetZSize ) ) + ( (double)kSize / 2.0 ) + 0.5 );
#endif

		/* 探索対象点群とモデル点群との距離評価値の合計値を算出 */
		sum += ( *DistanceField )[ k * ijSize + j * iSize + i ];

		/* 最小合計距離評価値よりも現在の合計距離評価値の方が大きい場合 */
		if( sum > *minSumofDistance ) break;	// ループを抜ける
	}


#ifdef COMMENTa
	cout << "DFSize: " << iSize << " " << jSize << " " << kSize << endl;
	cout << "TargetSize: " << targetXSize << " " << targetYSize << " " << targetZSize << endl;
	cout << "TargetGravity: " << gravX << " " << gravY << " " << gravZ << endl;
	cout << "SumofDistance: " << sum << endl;
#endif


	/* 探索対象点群とモデル点群との距離評価値の合計 */
	*SumofDistance = sum;
}