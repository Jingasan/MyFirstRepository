/***** ディスタンスフィールドの作成 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** ディスタンスフィールドの二値化 ***/
void BinarizationDFforPCDModel(
	int size,									// DF の分割数( DF のボクセルの分解能 )
	int *DFtemp,								// DF 作成のために一時的に保存する箱
	int **ClosestPointID,						// 最近点の ID
	struct ModelPointCloud** ModelPointCloud,	// 3次元点群
	int *PointSize,								// 3次元点群の数
	struct ModelInformation *ModelInf,			// 3次元点群の各種情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,	// 入力3次元点群
	double gravX,								// 3次元点群の X 方向の重心
	double gravY,								// 3次元点群の Y 方向の重心
	double gravZ								// 3次元点群の Z 方向の重心
){	// ここでボクセルとパッチとの距離が 0.5 以内のときに距離を 0 として DF に保存する

	int iSize, jSize, kSize;	// DF のサイズ
	int hiSize, hjSize, hkSize;	// DF の 1/2 のサイズ
	int qiSize, qjSize, qkSize;	// DF の 1/4 のサイズ
	int PCDPointSize;			// PCD のポイントサイズ
	double boxel;				// 1 voxel のサイズ
	double mingravX, mingravY, mingravZ;	// min と重心の和
	int ClosestPointIDnum = 0;				// ClosestPointID のナンバーを保存
	struct ModelPointCloud Surface;			// 3次元点群の情報
	vector< struct ModelPointCloud > ModelPointCloudtemp;	// 3次元点群の情報を一時的に保存
	

	/*** 高速化のために、色々格納しておく ***/

	/* 3次元点群の i, j, k 方向のサイズ */
	iSize = ModelInf->iSize;
	jSize = ModelInf->jSize;
	kSize = ModelInf->kSize;
	
	/* 3次元点群の i, j, k 方向の 1/2 のサイズ */
	hiSize = iSize / 2;
	hjSize = jSize / 2;
	hkSize = kSize / 2;
	
	/* 3次元点群の i, j, k 方向の 1/4 のサイズ */
	qiSize = iSize / 4;
	qjSize = jSize / 4;
	qkSize = kSize / 4;
	
	/* 1voexl のサイズ */
	boxel = 2 * ModelInf->max / DISTANCEFIELDSIZE;
	 
	/* 最小値の計算 */
	mingravX = ModelInf->Xmin + gravX;	// 3次元点群の X 方向の最小値の計算
	mingravY = ModelInf->Ymin + gravY;	// 3次元点群の Y 方向の最小値の計算
	mingravZ = ModelInf->Zmin + gravZ;	// 3次元点群の Z 方向の最小値の計算
	
	/* 3次元点群の数 */
	PCDPointSize = (int)cloud->points.size();


	/*** ボクセル内にある点を探索して値を 0 に初期化 ***/
	cout << "3次元点群を読み込み中……" << "\r" ;
	for( int pn = 0; pn < PCDPointSize; pn++ ){

		/* nanの点は読み飛ばす */
		//if( !_isnan( cloud->points[ pn ].x ) ){
		//モデルの範囲
		//割った誤差分プラスして回す??

			/* DF の重心から半分サイズまでの領域を探索 */
			for( int i = 1; i <= hiSize + 1; i++ ){
				for( int j = 1; j <= hjSize + 1; j++ ){
					for( int k = 1; k <= hkSize + 1; k++ ){

						/* 既に値が 0 なら以下に入る意味が無いのでスルー */
						if( DFtemp[ ( k + qkSize ) * iSize * jSize + ( j + qjSize ) * iSize + ( i + qiSize ) ] != 0 ){
							
							/* どのボクセルの中にあるかの調査 */
							if( cloud->points[ pn ].x * TOMETER >= mingravX + ( i - 1 ) * boxel && cloud->points[ pn ].x * TOMETER < mingravX + i * boxel ){
								if( cloud->points[ pn ].y * TOMETER >= mingravY + ( j - 1 ) * boxel && cloud->points[ pn ].y * TOMETER < mingravY + j * boxel ){
									if( cloud->points[ pn ].z * TOMETER >= mingravZ + ( k - 1 ) * boxel && cloud->points[ pn ].z * TOMETER < mingravZ + k * boxel ){
										
										/* DF を 0 に初期化 */
										DFtemp[ ( k + qkSize ) * iSize * jSize + ( j + qjSize ) * iSize + ( i + qiSize ) ] = 0;
										
										/* 最近点について処理 */
										( *ClosestPointID )[ ( k + qkSize ) *iSize * jSize + ( j + qjSize ) * iSize + ( i + qiSize ) ] = ClosestPointIDnum;
										ClosestPointIDnum++;
										
										/* ModelPointCloud を格納 */
										Surface.Xmodel = cloud->points[ pn ].x * TOMETER - gravX;
										Surface.Ymodel = cloud->points[ pn ].y * TOMETER - gravY;
										Surface.Zmodel = cloud->points[ pn ].z * TOMETER - gravZ;

										ModelPointCloudtemp.push_back( Surface );
										
										//あったらもう後のループは必要ない
										goto LOOPEXIT;
									} // if z
								} // if y
							} // if x
						} // if not zero
					} // for k
				} // for j
			} // for i
		//} // nan
LOOPEXIT:; //もう必要ないから次のループへと進む
	} // pn
   
	/* 評価される点の数 */
	*PointSize = ClosestPointIDnum;
	
	/* モデル点群のファイルに格納・モデルの色情報を格納 */
	( *ModelPointCloud ) = new struct ModelPointCloud[ ( *PointSize ) ];

	/* モデル点群を格納 */
	for( int i = 0; i < ClosestPointIDnum; i++ ){
		( *ModelPointCloud )[ i ].Xmodel = ModelPointCloudtemp[ i ].Xmodel;
		( *ModelPointCloud )[ i ].Ymodel = ModelPointCloudtemp[ i ].Ymodel;
		( *ModelPointCloud )[ i ].Zmodel = ModelPointCloudtemp[ i ].Zmodel;
	}
}


