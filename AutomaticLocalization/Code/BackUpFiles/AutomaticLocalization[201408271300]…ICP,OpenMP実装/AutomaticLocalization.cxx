/***** ディスタンスフィールドの作成 *****/


//【 コマンドライン引数 】
// 1. 探索対象点群ファイル
// 2. モデル点群ファイル

// 例 1：
// 1. Input/PointData/CompModel.pcd
// 2. Input/PointData/CompModel.pcd
// 例 2：
// 1. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd
// 2. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char* argv[] ){


	/* プログラム開始のコール */
	cout << "【 プログラム開始 】" << endl;

	/* 最大スレッド数の取得 */
	int threadsNum;
#pragma omp parallel
	{ threadsNum = omp_get_num_threads(); }	// { … } は「 #pragma omp parallel 」の隣に記述してはいけないので注意
	cout << "最大スレッド数：" << threadsNum << endl;

	/* コマンドライン引数読み込み失敗時 */
	if( argc < 3 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << " inputPointCloud1 inputPointCloud2" << endl;
		return EXIT_FAILURE;
	}
	

	/*** 変数の宣言 ***/
	const char * inputFilename1  = argv[ 1 ];			// 探索対象点群ファイル名
	const char * inputFilename2  = argv[ 2 ];			// モデル点群ファイル名
	
	/* DF 作成関連 */
	int DistanceFieldDevideSize = DISTANCEFIELDSIZE;	// DF の分割数( 既定値 )
	int *DistanceField = NULL;							// DF
	int *ClosestPointID = NULL;  						// 最近点の ID を保存
	int TargetPointCloudSize = 0;						// 3次元点群の数
	struct ModelPointCloud *TargetPointCloud = NULL;	// 3次元点群格納用変数
	struct ModelInformation TargetInformation;			// 3次元点群の各種情報
	int devideSize;										// DF の分割数( コマンドライン入力 )

	/* 全探索関連 */
	struct ModelPointCloud *modelPointCloud = NULL;		// 3次元点群格納用変数
	struct ModelInformation ModelInformation;			// 3次元点群の各種情報
	//Eigen::Matrix< double, 3, 3 > *R = NULL;			// 	
	int minSumofDistance = INT_MAX;

	vector< Eigen::Matrix< double, 3, 3 > > ER;
	vector< Eigen::Vector3d > Et;
	vector< double > DFval;
	ER.resize( 0 );
	Et.resize( 0 );
	DFval.resize( 0 );

	/* ディスタンスフィールドの分解能の指定 ⇒ Default：100 */
	cout << "ディスタンスフィールドの分解能を指定してください・・・" << endl;
	cout << "[ 0 以下 ⇒ 既定値:100 に設定, 1 以上 ⇒ 指定値に設定 ]" << endl;
	cout << ">> ";
	cin >> devideSize;
	if( devideSize > 0 ) DistanceFieldDevideSize = devideSize;	// DF の分解能の指定


	/*** 点群の読み込み： PCD データ ***/
	
	/* 探索対象点群データの読み込み */
	cout << ">>> 探索対象点群( PointXYZ )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud1( new pcl::PointCloud< pcl::PointXYZ > );	// 入力点群( PointXYZ )のメモリ確保
	if( pcl::io::loadPCDFile( inputFilename1, *cloud1 ) == -1 ){ cout << ">>> 探索対象点群データの読み込み失敗" << endl; return 1; }
	cout << ">>> 探索対象点群( PointXYZ )の読み込み OK" << endl;

	/* モデル点群データの読み込み */
	cout << ">>> モデル点群( PointXYZ )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud2( new pcl::PointCloud< pcl::PointXYZ > );	// 入力点群( PointXYZ )のメモリ確保
	if( pcl::io::loadPCDFile( inputFilename2, *cloud2 ) == -1 ){ cout << ">>> モデル点群データの読み込み失敗" << endl;return 1; }
	cout << ">>> モデル点群( PointXYZ )の読み込み OK" << endl;


	/*** ディスタンスフィールドの作成 ***/
	cout << "--- 【 CreateDistanceFieldforPCDModel関数 】DFの作成 ---" << endl;
	DWORD CreateDistanceFieldTimeStart = GetTickCount();	// DF 作成開始時間の取得
	CreateDistanceFieldforPCDModel(
		DistanceFieldDevideSize,	// DF の分割数( DF のボクセルの分解能 )
		&DistanceField,				// DF
		&ClosestPointID,			// 最近点の ID
		&TargetPointCloudSize,		// 3次元点群の数
		&TargetPointCloud,			// 3次元点群
		&TargetInformation,			// 3次元点群の各種情報
		cloud1						// 入力3次元点群
	);	// ディスタンスマップに格納されるのは点からの距離と対応する点と法線ベクトルのインデックスである	
	DWORD CreateDistanceFieldTimeEnd = GetTickCount();	// DF 作成終了時間の取得


	/*** 全探索 ***/
	cout << "--- 【 ExhaustiveSearch関数 】全探索 ---" << endl;
	DWORD ExhaustiveSearchTimeStart = GetTickCount();	// 全探索開始時間の取得
	ExhaustiveSearch(
		&DistanceField,		// DF
		&TargetInformation,	// 探索対象点群の各種情報
		&ModelInformation,	// モデル点群の各種情報
		&minSumofDistance,	// 探索対象点群とモデル点群との距離評価値の最小値
		cloud2,				// モデル点群
		cloud1,				// 探索対象点群
		&ER,				// 全探索結果の回転行列
		&Et,				// 全探索結果の並進ベクトル
		&DFval				// 全探索結果の DF 値
		//&R				// 回転行列 R
	);
	DWORD ExhaustiveSearchTimeEnd = GetTickCount();	// 全探索終了時間の取得






	/* 各種処理時間のコマンドライン出力 */
	cout << endl << "【 ResultTime 】" << endl;
	cout << "CreateDistanceFieldTime：" << (double)( CreateDistanceFieldTimeEnd - CreateDistanceFieldTimeStart ) / 1000 << " sec." << endl;
	cout << "ExhaustiveSearchTime：" << (double)( ExhaustiveSearchTimeEnd - ExhaustiveSearchTimeStart ) / 1000 << " sec." << endl;

	/* プログラム終了のコール */
	cout << "【 プログラム終了 】" << endl;

	return 0;
}
