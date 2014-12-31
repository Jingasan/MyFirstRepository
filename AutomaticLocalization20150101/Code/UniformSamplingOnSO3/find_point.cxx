/***** 回転空間の均等なサンプリング *****/


/*** インクルードファイル ***/

/* 作成ヘッダファイル */
#include"so3_sequence.h"



vector < double >find_point( int base_grid, long int point, long int level, long int healpix_point, double s1_point ){


	/* 変数の定義 */
	int position = point % 8;
	long int quo = 0;
	vector <double> Point;
	double interval = 30 / level;
	// the choosing of the order of the first resolution 4 points depends on which base healpix grid we are now dividing 

	if( base_grid == 6 || base_grid == 7 ){

		switch( position ){ //this position tells which of the eight points of the cube to consider

			case 0:
				healpix_point += 3;
				s1_point -= ( interval / 2 );
				break;
			case 1:
				healpix_point += 0;
				s1_point += ( interval / 2 );
				break;
			case 2: 
				healpix_point += 3;
				s1_point += ( interval / 2 );
				break;
			case 3:
				healpix_point += 0;
				s1_point -= ( interval / 2 );
				break;
			case 4:
				healpix_point += 2;
				s1_point -= ( interval / 2 );
				break;
			case 5:
				healpix_point += 1;
				s1_point += ( interval / 2 );
				break;
			case 6:
				healpix_point += 2;
				s1_point += ( interval / 2 );
				break;
			case 7:
				healpix_point += 1;
				s1_point -= ( interval / 2 );
				break;
		}
	}
	
	else if( base_grid == 3 || base_grid == 1 || base_grid == 9 || base_grid == 11 ){
		
		switch( position ){

			case 0:
				healpix_point += 3;
				s1_point -= ( interval / 2 );
				break;
			case 1:
				healpix_point += 0;
				s1_point += ( interval / 2 );
				break;
			case 2: 
				healpix_point += 3;
				s1_point += ( interval / 2 );
				break;
			case 3:
				healpix_point += 0;
				s1_point -= ( interval / 2 );
				break;
			case 4:
				healpix_point += 1;
				s1_point -= ( interval / 2 );
				break;
			case 5:
				healpix_point += 2;
				s1_point += ( interval / 2 );
				break;
			case 6:
				healpix_point += 1;
				s1_point += ( interval / 2 );
				break;
			case 7:
				healpix_point += 2;
				s1_point -= ( interval / 2 );
				break;
		}
	}

	else if( base_grid == 2 || base_grid == 0 || base_grid == 10 || base_grid == 8 ){

		switch( position ){

			case 0:
				healpix_point += 0;
				s1_point -= ( interval / 2 );
				break;
			case 1:
				healpix_point += 3;
				s1_point += ( interval / 2 );
				break;
			case 2: 
				healpix_point += 0;
				s1_point += ( interval / 2 );
				break;
			case 3:
				healpix_point += 3;
				s1_point -= ( interval / 2 );
				break;
			case 4:
				healpix_point += 1;
				s1_point -= ( interval / 2 );
				break;
			case 5:
				healpix_point += 2;
				s1_point += ( interval / 2 );
				break;
			case 6:
				healpix_point += 1;
				s1_point += ( interval / 2 );
				break;
			case 7:
				healpix_point += 2;
				s1_point -= ( interval / 2 );
				break;
		}
	}

	else if( base_grid == 4 || base_grid == 5 ){
		
		switch( position ){ 
			case 0:
				healpix_point += 0;
				s1_point -= ( interval / 2 );
				break;
			case 1:
				healpix_point += 3;
				s1_point += ( interval / 2 );
				break;
			case 2: 
				healpix_point += 0;
				s1_point += ( interval / 2 );
				break;
			case 3:
				healpix_point += 3;
				s1_point -= ( interval / 2 );
				break;
			case 4:
				healpix_point += 2;
				s1_point -= ( interval / 2 );
				break;
			case 5:
				healpix_point += 1;
				s1_point += ( interval / 2 );
				break;
			case 6:
				healpix_point += 2;
				s1_point += ( interval / 2 );
				break;
			case 7:
				healpix_point += 1;
				s1_point -= ( interval / 2 );
				break;
			
		}
	}
		
	quo = point / 8;
	
	if( quo == 0 ){

		long int nside = pow( 2, (double)level );	// 第1引数の第2引数乗( double 型 )
		double theta = 0, phi = 0, psi = 0;
		pix2ang_nest( nside, healpix_point, &theta, &phi );
		psi = s1_point * M_PI / 180;
		Point.resize( 0 );
		Point.push_back( theta );	
		Point.push_back( phi );	
		Point.push_back( psi );
		return Point;
	}

	else{
		return find_point( base_grid, quo - 1, level + 1, 4 * healpix_point, s1_point ); 
	}
}	
		
