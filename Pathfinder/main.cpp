#include "windows.h"

#include "main.h"
#include "bfs.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

/*

--- START ---
Steps: 1634
304939 305922 306905 306904 307887 308870 308871 308872 308873 309856 310839 311822 311823 312806 313789 313790 313791 313792 313793 313794 314777 315760 316743 317726 318709 319692 320675 321658 322641 322642 323625 324608 325591 326574 327557 328540 328541 329524 330507 331490 331491 331492 330509 330510 330511 331494 332477 332478 332479 333462 334445 335428 336411 336412 336413 336414 336415 337398 337399 337400 337401 338384 339367 339368 340351 341334 342317 342318 342319 343302 343303 343304 344287 345270 345269 345268 346251 347234 347235 348218 348219 348220 348221 347238 347239 347240 347241 347242 347243 347244 347245 346262 346263 346264 346265 346266 346267 345284 345285 345286 345287 345288 346271 346272 347255 348238 348239 349222 350205 350206 350207 350208 350209 351192 352175 353158 353157 354140 355123 356106 356107 357090 358073 358074 359057 360040 361023 362006 362989 363972 364955 365938 366921 367904 368887 368888 368889 369872 370855 371838 372821 373804 374787 374788 374789 374790 374791 374792 374793 375776 375777 376760 377743 378726 378725 378724 379707 380690 381673 381672 382655 383638 384621 385604 386587 387570 388553 389536 390519 391502 392485 392484 393467 394450 395433 395434 395435 395436 395437 396420 396421 396422 397405 398388 399371 400354 401337 401338 402321 403304 403303 404286 405269 405270 405271 405272 405273 406256 406257 407240 407241 407242 408225 409208 409207 410190 411173 411174 411175 411176 411177 412160 413143 414126 415109 415110 415111 415112 415113 415114 415115 415116 415117 415118 415119 414136 414137 414138 413155 412172 411189 411190 411191 411192 410209 409226 409227 409228 409229 410212 410213 410214 411197 412180 412181 412182 413165 414148 415131 415132 415133 415134 416117 416118 416119 416120 416121 417104 417105 418088 419071 419072 419073 419074 419075 419076 419077 419078 419079 418096 418097 418098 417115 416132 416133 416134 416135 416136 416137 416138 416139 416140 416141 417124 417125 417126 416143 416144 416145 416146 416147 415164 414181 414182 414183 415166 416149 417132 418115 419098 420081 420082 420083 421066 421067 421068 422051 423034 424017 425000 425001 425002 425985 426968 426969 426970 425987 425988 425005 425006 425007 425008 425009 425992 426975 426976 426977 427960 428943 429926 430909 431892 432875 432876 433859 434842 435825 435826 436809 436810 437793 437794 437795 436812 436813 436814 436815 437798 438781 438782 439765 439766 439767 438784 438785 438786 438787 439770 440753 441736 442719 442720 442721 442722 442723 442724 442725 441742 440759 440760 440761 441744 442727 443710 444693 445676 446659 446658 446657 447640 448623 449606 449607 449608 449609 450592 451575 452558 452559 452560 453543 454526 455509 455510 455511 455512 455513 455514 455515 455516 456499 457482 458465 459448 460431 460432 461415 461416 461417 461418 460435 459452 459453 459454 460437 460438 461421 461422 461423 461424 461425 461426 462409 463392 464375 464376 465359 466342 466343 467326 467327 467328 467329 466346 465363 464380 463397 463398 463399 464382 464383 464384 464385 464386 465369 465370 465371 464388 464389 464390 464391 464392 464393 464394 464395 464396 465379 466362 466363 467346 468329 469312 470295 471278 471277 472260 473243 473242 473241 473240 474223 475206 476189 477172 478155 479138 479137 480120 481103 481104 482087 483070 484053 485036 486019 486020 487003 487986 488969 489952 489953 489954 490937 491920 491921 492904 493887 494870 495853 496836 497819 498802 498803 499786 500769 501752 502735 503718 504701 505684 505683 506666 507649 507650 507651 507652 507653 508636 509619 510602 510603 510604 511587 512570 512571 513554 514537 515520 516503 517486 517485 518468 519451 520434 520433 520432 521415 522398 522397 522396 522395 522394 522393 523376 523375 524358 525341 526324 527307 528290 529273 529274 530257 531240 532223 533206 534189 535172 536155 537138 538121 539104 539105 539106 539107 539108 539109 540092 541075 542058 543041 543042 543043 544026 545009 545992 545993 545994 545995 545012 545013 545014 545015 545016 545017 545018 545019 545020 545021 545022 546005 546988 547971 548954 549937 549938 549939 548956 547973 546990 546991 546992 546993 546994 546995 547978 547979 547980 547981 548964 548965 548966 549949 549950 549951 549952 549953 549954 549955 549956 549957 549958 549959 549960 550943 551926 551927 552910 553893 553894 554877 555860 555861 555862 556845 556846 556847 556848 556849 556850 556851 557834 558817 559800 560783 561766 562749 563732 563731 564714 565697 565698 565699 565700 565701 565702 565703 565704 566687 566688 566689 567672 567673 568656 569639 569640 569641 569642 570625 570626 571609 572592 573575 574558 574559 574560 574561 574562 574563 574564 575547 575548 575549 576532 576533 576534 577517 578500 579483 580466 581449 581450 581451 581452 582435 582436 582437 583420 583421 583422 582439 582440 582441 582442 582443 583426 584409 584410 584411 584412 585395 586378 587361 587360 588343 589326 590309 591292 592275 592274 592273 592272 593255 594238 595221 595222 595223 594240 594241 594242 594243 594244 595227 595228 595229 595230 596213 596214 596215 596216 597199 598182 598183 599166 600149 600150 600151 601134 601135 601136 601137 601138 601139 602122 603105 604088 605071 606054 607037 608020 608021 608022 608023 608024 608025 608026 608027 608028 608029 608030 608031 608032 608033 608034 608035 608036 609019 610002 610985 610986 611969 611970 611971 611972 611973 610990 610991 610008 609025 608042 607059 606076 606077 606078 607061 608044 609027 610010 610993 611976 612959 613942 614925 615908 616891 617874 617873 617872 617871 617870 618853 619836 620819 620820 620821 621804 622787 623770 624753 625736 625735 625734 626717 627700 628683 629666 630649 631632 631631 632614 633597 634580 634579 635562 636545 636544 636543 636542 636541 636540 636539 636538 637521 638504 638505 639488 640471 641454 642437 643420 643419 643418 643417 643416 644399 645382 646365 647348 647347 647346 647345 648328 649311 650294 650295 651278 652261 652262 653245 654228 655211 656194 656193 657176 658159 658158 659141 660124 661107 662090 663073 664056 665039 666022 667005 667006 667989 667990 667991 667992 668975 668976 668977 668978 667995 667996 667013 667014 667015 666032 666033 666034 666035 666036 666037 667020 668003 668986 668987 668988 668989 669972 669973 669974 669975 669976 669977 669978 668995 668996 668013 667030 667031 667032 667033 667034 666051 666052 666053 666054 666055 666056 666057 666058 665075 665076 665077 666060 666061 666062 666063 666064 666065 666066 667049 668032 668033 668034 669017 670000 669999 669998 670981 671964 671965 671966 671967 672950 673933 674916 675899 676882 677865 678848 679831 680814 681797 682780 682779 683762 684745 685728 686711 687694 687695 688678 688679 688680 688681 688682 689665 690648 691631 692614 693597 694580 694581 694582 694583 694584 694585 695568 695569 695570 695571 695572 695573 695574 695575 695576 695577 695578 695579 695580 696563 697546 698529 699512 699513 700496 701479 701478 702461 703444 703445 703446 703447 704430 704431 705414 705415 705416 705417 706400 707383 707384 707385 707386 708369 709352 709353 710336 711319 711320 712303 713286 714269 715252 716235 717218 718201 719184 720167 721150 721151 721152 720169 719186 718203 718204 718205 718206 718207 718208 718209 718210 719193 720176 720175 721158 722141 722142 723125 724108 724109 725092 725093 725094 726077 727060 728043 729026 729025 730008 730991 731974 731975 731976 732959 732960 732961 732962 732963 732964 731981 730998 730999 731000 731001 731002 731985 731986 731987 732970 733953 734936 734937 735920 736903 737886 738869 738870 739853 739854 740837 741820 742803 742802 742801 743784 744767 745750 746733 747716 748699 749682 750665 750666 751649 752632 752633 752634 751651 750668 749685 748702 747719 746736 746737 746738 746739 745756 745757 744774 744775 744776 744777 745760 745761 745762 745763 745764 745765 745766 746749 746750 746751 746752 746753 746754 747737 748720 748721 749704 750687 751670 751669 752652 753635 754618 755601 756584 756585 757568 758551 759534 759535 759536 759537 760520 760521 761504 762487 762486 763469 764452 764451 765434 766417 767400 768383 769366 770349 771332 772315 773298 774281 774280 775263 775262 775261 776244 777227 778210 779193 779194 779195 780178 781161 782144 782143 783126 784109 784110 785093 785094 785095 784112 783129 783130 783131 783132 783133 784116 784117 784118 785101 786084 786085 787068 788051 789034 789035 789036 789037 789038 789039 789040 790023 791006 791989 791990 791991 791008 791009 790026 789043 789044 789045 789046 789047 789048 789049 789050 789051 789052 789053 789054 789055 789056 789057 790040 791023 792006 792989 793972 794955 795938 796921 796922 797905 797906 798889 799872 799873 800856 800857 800858 800859 800860 800861 800862 800863 800864 800865 801848 801849 801850 802833 803816 803817 803818 803819 803820 803821 803822 803823 803824 804807 805790 806773 807756 807755 808738 809721 809722 810705 811688 812671 813654 813655 814638 814639 814640 814641 815624 816607 817590 818573 818574 818575 817592 817593 817594 817595 817596 817597 817598 817599 818582 818583 818584 817601 816618 815635 815636 815637 816620 816621 817604 818587 818588 819571 819572 819573 820556 820557 820558 820559 820560 820561 820562 820563 820564 820565 819582 819583 819584 820567 821550 821551 821552 822535 823518 824501 824502 824503 825486 826469 826470 826471 826472 826473 826474 826475 826476 826477 825494 824511 823528 822545 821562 821561 821560 822543 822542 822541 822540 822539 821556 820573 819590 818607 818608 818609 818610 818611 818612 818613 817630 817631 817632 817633 816650 816651 816652 815669 814686 814687 814688 814689 815672 815673 815674 814691 814692 814693 814694 814695 815678 815679 815680 815681 814698 813715 812732 811749 811750 811751 811752 811753 811754 811755 811756 811757 811758 811759 811760 811761 810778 810779 810780 810781 811764 811765 811766 811767 811768 811769 811770 812753 813736 813737 813738 813739 814722 815705 815706 816689 817672 817673 817674 817675 817676 817677 818660 819643 819644 819645 819646 819647 820630 821613 822596 823579 824562 825545 826528 827511 828494 829477 830460 831443 831444 832427 833410 833411 833412 833413 833414 834397 834398 834399 834400 834401 834402 834403 834404 835387 836370 837353 838336 839319 839320 839321 839322 839323 839324 840307 841290 842273 842274 842275 842276 843259 844242 845225 845226 845227 845228 845229 844246 844247 844248 844249 845232 845233 846216 846217 846218 845235 844252 844253 844254 844255 843272 843273 843274 844257 845240 845241 846224 846225 846226 846227 847210 848193 849176 850159 851142 852125 852126 852127 853110 854093 855076 855077 855078 855079 856062 857045 858028 859011 859010 859993 860976 860977 860978 861961 861962 862945 862946 862947 862948 862949 862950 862951 863934 864917 865900 865901 865902 866885 866886 866887 866888 867871 867872 868855 868856 868857 868858 868859 868860 868861 869844 869845 870828 870829 870830 870831 870832 870833 870834 870835 871818 871819 871820 872803 872804 872805 873788 874771 875754 876737 877720 877721 877722 878705 878706 879689 880672 880673 880674 880675 880676 880677 880678 880679 880680 880681 879698 879699 879700 879701 880684 881667 882650 882651 883634 884617 885600 886583 887566
--- END ---

*/
int main()
{
	char PrintBuffer[256];
	{
		OutputDebugString("\n--- START ---\n");
		unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
		int pOutBuffer[12];
		int nSteps = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
		sprintf_s(PrintBuffer, "Steps: %d\n", nSteps);
		OutputDebugString( PrintBuffer );
		for (int i = 0; i < nSteps; ++i)
		{
			sprintf_s(PrintBuffer, "%d ", pOutBuffer[i]);
			OutputDebugString(PrintBuffer);
		}
		OutputDebugString("\n--- END ---\n");
	}

	{
		OutputDebugString("\n--- START ---\n");
		unsigned char pMap[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
		int pOutBuffer[7];
		int nSteps = FindPath(2, 0, 0, 2, pMap, 3, 3, pOutBuffer, 7);
		sprintf_s(PrintBuffer, "Steps: %d\n", nSteps);
		OutputDebugString(PrintBuffer);
		for (int i = 0; i < nSteps; ++i)
		{
			sprintf_s(PrintBuffer, "%d ", pOutBuffer[i]);
			OutputDebugString(PrintBuffer);
		}
		OutputDebugString("\n--- END ---\n");
	}

	{
		OutputDebugString("\n--- START ---\n");

		using namespace std;

		streampos size;
	
		ifstream file("maze.txt");
		if (file.is_open())
		{
			size = file.tellg();
			std::vector<unsigned char> Map;
			Map.reserve(1024 * 1024);
			while (!file.eof())
			{
				int nGet = file.get();
				if (nGet == '#')
				{
					Map.push_back(0);
				}
				else if (nGet == '.')
				{
					Map.push_back(1);
				}
				else
				{
					// do nothing
				}
			}
			file.close();
			int nMapSize = Map.size();
			if (nMapSize == 983 * 991)
			{
				int* pOutBuffer = new int[10000];
				int nSteps;
				for (int i = 0; i < 50; ++i)
				{
					nSteps = FindPath(900, 902, 210, 310, &Map[0], 983, 991, pOutBuffer, 10000);
					nSteps = FindPath(210, 310, 900, 902, &Map[0], 983, 991, pOutBuffer, 10000);
				}
				sprintf_s(PrintBuffer, "Steps: %d\n", nSteps);
				OutputDebugString(PrintBuffer);
				for (int i = 0; i < nSteps; ++i)
				{
					sprintf_s(PrintBuffer, "%d ", pOutBuffer[i]);
					OutputDebugString(PrintBuffer);
				}
				OutputDebugString("\n--- END ---\n");
				delete[] pOutBuffer;
			}
		}

	}

	const char* pMap = "0000100001"
		"0000100001"
		"0000100001"
		"0000100001"
		"0000100001";

	std::string Out;
	const int nWidth = 10;
	const char* pCurrent = pMap;
	int nLine = nWidth;
	while (*pCurrent)
	{
		if (nLine == 0)
		{
			Out += '\n';
			nLine = nWidth;
		}
		Out += *pCurrent;
		--nLine;
		++pCurrent;
	}
	OutputDebugString("\n--- START ---\n");
	OutputDebugString( Out.c_str() );
	OutputDebugString( "\n--- END ---\n");
	return 0;
}
