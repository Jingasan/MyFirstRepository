/***** Automatic Localization *****/

// 【 Program Pipeline 】
// Step1 ディスタンスフィールドの作成 
// Step2 全探索
// Step3 最適化：Marquart or ICP


//【 コマンドライン引数 】
// 1. 探索対象点群ファイル
// 2. モデル点群ファイル

// 例 1：
// 1. Input/PointData/CompModel.pcd
// 2. Input/PointData/CompModel.pcd

// 例 2：探索対象点群とモデル点群(位置姿勢変化無)の位置合わせ
// 1. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd
// 2. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd

// 例 3：探索対象点群とモデル点群(位置姿勢変化有)の位置合わせ
// 1. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// 例 4：探索対象点群とモデル点群(位置姿勢変化有)の位置合わせ
// 1. Output/TransformPointCloud/LiverR20/Ra200Rb205Rc210Tx0Ty0Tz0/0001.pcd
// 2. Input/PointData/ModelData/LiverR20/0001.pcd

// 例 5：RandomNoise 追加後の探索対象点群とモデル点群の位置合わせ
// 1. Output/NoisyPointData/ModelData/LiverR20/0001N1000.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// 例 6：GaussianNoise 付加後の探索対象点群とモデル点群の位置合わせ
// 1. Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// 例 7：RandomNoise 追加 & GaussianNoise 付加後の探索対象点群とモデル点群の位置合わせ
// 1. Output/NoisyPointData/ModelData/LiverR20/0001_1GaussianN1000.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// 例 8：RandomNoise 追加 & GaussianNoise 付加後の探索対象点群とモデル点群の位置合わせ
// 1. Output/NoisyPointData/ModelData/LiverR20/0001_5GaussianN1000.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd


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
		cerr << argv[ 0 ] << endl;
		cerr << "1. InputPointCloud1" << endl;
		cerr << "2. InputPointCloud2" << endl;
		system( "pause" );
		return EXIT_FAILURE;
	}else{
		cout << ">> InputTargetData : " << argv[ 1 ] << endl;
		cout << ">> InputModelData : " << argv[ 2 ] << endl;
	}
	

	/*** 変数の宣言 ***/
	const char * inputFilename1 = argv[ 1 ];			// 探索対象点群ファイル名
	const char * inputFilename2 = argv[ 2 ];			// モデル点群ファイル名
	
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
	vector< Eigen::Matrix< double, 3, 3 > > ER;	// 全探索結果の回転行列
	vector< Eigen::Vector3d > Et;				// 全探索結果の並進ベクトル
	vector< double > DFval;						// 全探索結果のDF合計値

	/* メモリの確保 */
	ER.resize( 0 ); Et.resize( 0 ); DFval.resize( 0 );

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
#ifdef CREATEDISTANCEFIELD
	cout << "--- 【 CreateDistanceFieldforPCDModel関数 】DFの作成 ---" << endl;
	DWORD CreateDistanceFieldTimeStart = GetTickCount();	// DF 作成開始時間の取得
	CreateDistanceFieldforPCDModel(
		DistanceFieldDevideSize,	// DF の分割数( DF のボクセルの分解能 )
		&DistanceField,				// DF
		&ClosestPointID,			// 最近点の ID
		&TargetPointCloudSize,		// 点群数
		&TargetPointCloud,			// 点群
		&TargetInformation,			// 点群の各種情報
		cloud1						// 入力点群
	);	// ディスタンスマップに格納されるのは点からの距離と対応する点と法線ベクトルのインデックスである	
	DWORD CreateDistanceFieldTimeEnd = GetTickCount();	// DF 作成終了時間の取得
#endif

	/*** 全探索 ***/
#ifdef EXHAUSTIVESEARCH
	cout << "--- 【 ExhaustiveSearch関数 】全探索 ---" << endl;
	DWORD ExhaustiveSearchTimeStart = GetTickCount();	// 全探索開始時間の取得
	ExhaustiveSearch(
		DistanceFieldDevideSize,	// DF の分割数( DF のボクセルの分解能 )
		&DistanceField,				// DF
		&TargetInformation,			// 探索対象点群の各種情報
		&ModelInformation,			// モデル点群の各種情報
		&minSumofDistance,			// 探索対象点群とモデル点群との距離評価値の最小値
		cloud2,						// モデル点群
		cloud1,						// 探索対象点群
		&ER,						// 全探索結果の回転行列
		&Et,						// 全探索結果の並進ベクトル
		&DFval						// 全探索結果の DF 値
		//&R	// 回転行列 R
	);
	DWORD ExhaustiveSearchTimeEnd = GetTickCount();	// 全探索終了時間の取得
#endif

	/*** マーカート法による最適化 ***/
#ifdef MARQUARDT
	cout << "--- 【 LevMarOptimization関数 】最適化 ---" << endl;
	DWORD LevMarTimeStart = GetTickCount();	// LevenbergMarquardt法による最適化の開始時間の取得
	LevMarOptimization(
		&DistanceField,				// DF
		&ClosestPointID,			// 最近点の ID
		&TargetInformation,			// 探索対象点群の各種情報
		&ModelInformation,			// モデル点群の各種情報
		cloud2,						// モデル点群
		cloud1,						// 探索対象点群
		&ER,						// 全探索結果の回転行列
		&Et,						// 全探索結果の並進ベクトル
		&DFval						// 全探索結果の DF 値
	);
	DWORD LevMarTimeEnd = GetTickCount();	// LevenbergMarquardt法による最適化の終了時間の取得
#endif

	/*** ICP を用いた2点群の最適化 ***/
#ifdef ITERATIVECLOSESTPOINT
	cout << "--- 【 ICPOptimization関数 】最適化 ---" << endl;
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM( new pcl::PointCloud< pcl::PointXYZRGB > );		// 位置合わせ後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedIcpCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// ICP 後の統合点群
	DWORD ICPTimeStart = GetTickCount();	// ICPアルゴリズムによる最適化の開始時間の取得
	ICPOptimization(
		cloud2,						// モデル点群
		cloud1,						// 探索対象点群
		newCloudM,					// 最適化後のモデル点群
		mergedIcpCloud,				// 最適化後のモデル点群と探索対象点群の統合結果
		&TargetInformation,			// 探索対象点群の各種情報
		&ModelInformation,			// モデル点群の各種情報
		&ER,						// 全探索結果の回転行列
		&Et							// 全探索結果の並進ベクトル
	);
	DWORD ICPTimeEnd = GetTickCount();	// ICPアルゴリズムによる最適化の終了時間の取得
#endif



	/* 各種処理時間のコマンドライン出力 */
	cout << endl << "【 ResultTime 】" << endl;
	double CreateDistanceFieldTime = -1; double ExhaustiveSearchTime = -1; double LevMarTime = -1; double ICPTime = -1;
#ifdef CREATEDISTANCEFIELD
	CreateDistanceFieldTime = (double)( CreateDistanceFieldTimeEnd - CreateDistanceFieldTimeStart ) / 1000;
	cout << "CreateDistanceFieldTime：" << CreateDistanceFieldTime << " sec." << endl;
#endif
#ifdef EXHAUSTIVESEARCH
	ExhaustiveSearchTime = (double)( ExhaustiveSearchTimeEnd - ExhaustiveSearchTimeStart ) / 1000;
	cout << "ExhaustiveSearchTime：" << ExhaustiveSearchTime << " sec." << endl;
#endif
#ifdef MARQUARDT
	LevMarTime = (double)( LevMarTimeEnd - LevMarTimeStart ) / 1000;
	cout << "LevenbergMarquardtTime：" << LevMarTime << " sec." << endl;
#endif
#ifdef ITERATIVECLOSESTPOINT
	ICPTime = (double)( ICPTimeEnd - ICPTimeStart ) / 1000;
	cout << "ICPTime：" << ICPTime << " sec." << endl;
#endif
#ifdef OUTPUTEXECUTIONTIME
	cout << ">> 実行結果出力中" << "\r";
	FILE *outputFp1;
	if( ( outputFp1 = fopen( ExecutionTimeFileName, "w" ) ) == NULL ){ cout << "Caution!: " << ExecutionTimeFileName << " open error" << endl; return 1; }
	fprintf( outputFp1, "CreateDistanceFieldTime[sec],%f\nExhaustiveSearchTime[sec],%f\nLevenbergMarquardtTime[sec],%f\nICPTime[sec],%f\n", CreateDistanceFieldTime, ExhaustiveSearchTime, LevMarTime, ICPTime );
	fclose( outputFp1 );
	cout << ">> 実行結果出力 OK!" << endl;
#endif

	/* プログラム終了のコール */
	cout << endl << "【 プログラム終了 】" << endl;

	return 0;
}
