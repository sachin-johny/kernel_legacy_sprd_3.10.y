#******************************************************************************* 
#* Copyright (C) 2012-2013, Trcaicio Co., Ltd.
#* All rigths reserved
#*
#* Filename : trace_tool.pl
#* Abstract : 
#*
#* Author   : Ke.Li
#* Version  : 1.00
#*
#* Revison Log : 
#*     2012/09/06, Ke.Li: Create this file
#*
#* CVS Log :
#*     $Id$
#*******************************************************************************


#******************************
#     Include Header File
#******************************

use strict;
use warnings;


#******************************
#   Global Variable Declare
#******************************

# public global variable




# include external file
require 'p1.pl';
require 'p2.pl';
require 'p3.pl';
require 'zhuyg.pl';



# internal global variable

my @g_file_folder = ();
my @g_file_route = ();
my @g_valid_file = ();
my @g_display_config = (1,1,1,1,1,1,1);
my $g_source_code_folder = 'code_backup';
my $g_backup_folder = '';
my $g_tmp_backup_folder = 'last_backup';
my $g_decode_table = 'decode_table';
my $g_progress_value = 0;
my $g_progress_type = 0;
my $g_progress_standard_value = 0;
my @g_replace_match_str = ('TROUT_FUNC_ENTER','TROUT_FUNC_EXIT','TROUT_DBG',
                           'TROUT_TX_DBG','TROUT_RX_DBG','TROUT_INT_DBG',
                           'TROUT_PRINT');

my $g_file_name;
my $g_file_id = 1;
my $g_function_name;
my $g_function_id = 0;
my $top_print = 1;

#******************************
#        Main Process
#******************************

main_process();


#******************************
#   Main Function Describe
#******************************

#******************************************************************** 
#*
#* Function Name : main_process
#*
#* Description   : Display main menu option to user, wait user select
#*                 the option number.
#*
#* Globals       : None
#*
#* Inputs        : None
#*
#* Outputs       : None
#*
#* Returns       : None
#*
#********************************************************************
sub main_process
{
	my $result = 0;
	
	main_function_shortcut();
	system('clear');
	print_to_screen("====== Welcome to use trace tool ======",3,3,3);
	print_to_screen("       1: Encode log.",3,0,2);
	print_to_screen("       2: Decode log.",3,0,2);
	print_to_screen("       3: Analyse log.",3,0,2);
	print_to_screen("          Select: ",3,0,0);
	my $user_select_value = 0;
	while(1)
	{
		$user_select_value = gather_input_number(1,3);
		if($user_select_value eq (-1))
		{
			print_to_screen("          Input invalid!: ",3,2,2);
			print_to_screen("          Select: ",3,0,0);
		}
		else
		{
			last;
		}
	}
	if($user_select_value eq 1)
	{
		if(-e $g_source_code_folder)
		{
			if(-e $g_tmp_backup_folder)
			{
				system("rm -rf $g_tmp_backup_folder");
				mkdir($g_tmp_backup_folder);
				$g_backup_folder = $g_tmp_backup_folder;
			}
		}
		else
		{
				mkdir($g_source_code_folder);
				$g_backup_folder = $g_source_code_folder;
		}
		$result = main_function_encode_log(0);
	}
	elsif($user_select_value eq 2)
	{
		$result = main_function_decode_log();
	}
	elsif($user_select_value eq 3)
	{
		$result = main_function_analyse_log();
	}
	else
	{
		print_to_screen("Bug: main_process input_value = $user_select_value",3,3,2);
	}
	
	if($result eq -1)
	{
		print_to_screen("====== execute error! ======",3,2,3);
		exit;
	}
	else
	{
		print_to_screen("====== execute complete! ======",3,2,3);
		exit;
	}
	
}


sub main_function_shortcut
{
	my $result = 0;
	my $argv_offset = 1;
	
	if(defined $ARGV[0])
	{
		if($ARGV[0] eq 1)
		{
			if(-e $g_source_code_folder)
			{
				if(-e $g_tmp_backup_folder)
				{
					system("rm -rf $g_tmp_backup_folder");
				}
				$g_backup_folder = $g_tmp_backup_folder;
				mkdir($g_tmp_backup_folder);
			}
			else
			{
				mkdir($g_source_code_folder);
				$g_backup_folder = $g_source_code_folder;
			}
			while($argv_offset)
			{
				if(defined $ARGV[$argv_offset])
				{
					if(-e "../$ARGV[$argv_offset]")
					{
						system("cp -rf ../$ARGV[$argv_offset] $g_backup_folder/$ARGV[$argv_offset]");
						push(@g_file_folder,"$g_backup_folder/$ARGV[$argv_offset]");
						$argv_offset++;
					}
					else
					{
						print_to_screen("       can't file folder ../$ARGV[$argv_offset]",3,2,3);
						exit;
					}
				}
				else
				{
					last;
				}
			}
			if($argv_offset > 1)
			{
				$result = main_function_encode_log(1);
			}
			else
			{
				return;
			}
		}
		elsif($ARGV[0] eq 2)
		{
			for(my $i = 1; $i < 8; $i++)
			{
				if(defined $ARGV[$i] and $ARGV[$i] eq 0)
				{
					$g_display_config[$i-1] = 0;
				}
			}
			$result = main_function_decode_log();
		}
		elsif($ARGV[0] eq 3)
		{
			$result = main_function_analyse_log();
		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}

	if($result eq -1)
	{
		print_to_screen("====== execute error! ======",3,2,3);
		exit;
	}
	else
	{
		print_to_screen("====== execute complete! ======",3,2,3);
		exit;
	}
	
}


sub main_function_encode_log
{
	my ($shortcut_en) = @_;

	@_ = ();

	my $folder_num = 1;
	my $input_information;

	if($shortcut_en eq 0)
	{
		menu_encode_log();
	}

	foreach my $tmp_folder(@g_file_folder)
	{
		my @tmp_file_route = get_c_code_file($tmp_folder);
		if(@tmp_file_route eq 0)
		{
			print_to_screen("       scan no any file!",3,2,2);
			return (-1);
		}
		else
		{
			push(@g_file_route,@tmp_file_route);
		}
	}
	if(get_valid_file() eq (-1))
	{
		return (-1);
	}

	open(my $TABLE,">$g_decode_table");
	close($TABLE);
	
	my $tmp_file_num = @g_valid_file;
	$g_progress_value = 20;
	if($tmp_file_num > 80)
	{
		$g_progress_type = 1;
		if($tmp_file_num % 80)
		{
			$g_progress_standard_value = int($tmp_file_num / 80) + 1;
		}
		else
		{
			$g_progress_standard_value = int($tmp_file_num / 80);
		}
	}
	else
	{
		$g_progress_type = 0;
		$g_progress_standard_value = int(80 / $tmp_file_num);
	}
	$tmp_file_num = 0;
	if($top_print eq 1)
	{
		system("clear");
		print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
	}
	
	foreach my $tmp_file(@g_valid_file)
	{
		my $create_file = $tmp_file;
		$create_file =~ s/$g_backup_folder/../o;
		my $result = code_line_deal($tmp_file,$create_file);
		if($result eq (-1))
		{
			print_to_screen("       code_line_deal error!",3,2,2);
			exit;
		}
		
		if($g_progress_type eq 0)
		{
			$g_progress_value += $g_progress_standard_value;
			if($top_print eq 1)
			{
				system("clear");
				print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
			}
		}
		else
		{
			$tmp_file_num++;
			if($tmp_file_num eq $g_progress_standard_value)
			{
				$tmp_file_num = 0;
				$g_progress_value++;
				if($top_print eq 1)
				{
					system("clear");
					print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
				}
			}
		}
	}
	if($top_print eq 1)
	{
		system("clear");
		print_to_screen("       please waiting ... (100%)\n",0,0,2);
	}
	
}

sub main_function_decode_log
{
	return log_restore(@g_display_config);
	
}

sub main_function_analyse_log
{
	if(-e "result")
	{
		system("rm -rf result");
	}
	mkdir("result");

}


#******************************
#  Private Function Describe
#******************************


#******************************************************************** 
#*
#* Function Name : print_to_screen
#*
#* Description   : Control screen print format.
#*
#* Globals       : None
#*
#* Inputs        : print_str    --- Print content.
#*                 front_offset --- space number at print line.
#*                 front_line   --- null line number in front of
#*                                  print line.
#*                 behind_line  --- null line number in behind of
#*                                  print line.
#*
#* Outputs       : None
#*
#* Returns       : None
#*
#********************************************************************
sub print_to_screen
{
	my ($print_str,$front_offset,$front_line,$behind_line) = @_;

	@_ = ();

	while($front_line--)
	{
		print("\n");
	}
	while($front_offset--)
	{
		print(" ");
	}
	print("$print_str");
	while($behind_line--)
	{
		print("\n");
	}
	
}


#******************************************************************** 
#*
#* Function Name : gather_input_number
#*
#* Description   : Gather menu option number valid check.
#*
#* Globals       : None
#*
#* Inputs        : option_min --- option min valid number.
#*               : option_max --- option max valid number.
#*
#* Outputs       : None
#*
#* Returns       : (-1)  --- Input number is invalid.
#*                 other --- Input number value.
#*
#********************************************************************
sub gather_input_number
{
	my ($option_min,$option_max) = @_;

	@_ = ();

	my $input_value = <STDIN>;
	chomp $input_value;
	$input_value = int($input_value);
	if($input_value < $option_min or $input_value > $option_max)
	{
		return (-1);
	}
	else
	{
		return $input_value;
	}
	
}


#******************************************************************** 
#*
#* Function Name : gather_input
#*
#* Description   : Gather user input information.
#*
#* Globals       : None
#*
#* Inputs        : None
#*
#* Outputs       : None
#*
#* Returns       : any --- User input.
#*
#********************************************************************
sub gather_input
{
	my $input_information = <STDIN>;
	chomp $input_information;
	return $input_information;
	
}


sub menu_encode_log
{
	my $folder_num = 1;
	
	print_to_screen("====== input dest folder route ======",3,2,2);
	print_to_screen("====== input \"ok\" to end folder set ======",3,2,2);
	while($folder_num)
	{
		print_to_screen("       folder $folder_num = ",3,0,0);
		my $input_information = gather_input();
		my $input_tmp = $input_information;
		$input_tmp =~ s/\s+//g;
		print_to_screen("",0,0,1);
		if($input_information eq 'ok' or $input_information eq 'OK')
		{
			if(@g_file_folder eq 0)
			{
				print_to_screen("       you don't input any folder!",3,2,3);
				$folder_num = 1;
			}
			else
			{
				return;
			}
		}
		elsif($input_tmp ne '')
		{
			if(-e "../$input_tmp")
			{
				if(!(-e $g_backup_folder))
				{
					mkdir($g_backup_folder);
				}
				system("cp ../$input_tmp $g_backup_folder/$input_tmp");
				push(@g_file_folder,"$g_backup_folder/$input_tmp");
				$folder_num++;
			}
			else
			{
				print_to_screen("       can't file folder ../$input_tmp",3,2,3);
			}
		}
	}

}


sub get_valid_file
{
	my $match_en = 0;

	my $tmp_file_num = @g_file_route;
	$g_progress_value = 0;
	if($tmp_file_num > 20)
	{
		$g_progress_type = 1;
		if($tmp_file_num % 20)
		{
			$g_progress_standard_value = int($tmp_file_num / 20) + 1;
		}
		else
		{
			$g_progress_standard_value = int($tmp_file_num / 20);
		}
	}
	else
	{
		$g_progress_type = 0;
		$g_progress_standard_value = int(20 / $tmp_file_num);
	}
	$tmp_file_num = 0;
	if($top_print eq 1)
	{
		system("clear");
		print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
	}
	foreach my $tmp_route(@g_file_route)
	{
		open(my $file_handle,"$tmp_route") || die "       can't open $tmp_route\n";
		$match_en = 0;
		while(my $tmp_line = <$file_handle>)
		{
			foreach my $tmp_content(@g_replace_match_str)
			{
				if($tmp_line =~ /$tmp_content/)
				{
					$match_en = 1;
					last;
				}
			}
			if($match_en eq 1)
			{
				push @g_valid_file, $tmp_route;
				last;
			}
		}
		close($file_handle);
		if($g_progress_type eq 0)
		{
			$g_progress_value += $g_progress_standard_value;
			if($top_print eq 1)
			{
				system("clear");
				print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
			}
		}
		else
		{
			$tmp_file_num++;
			if($tmp_file_num eq $g_progress_standard_value)
			{
				$tmp_file_num = 0;
				$g_progress_value++;
				if($top_print eq 1)
				{
					system("clear");
					print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
				}
			}
		}
		next;
		
	}
	if($top_print eq 1)
	{
		system("clear");
		print_to_screen("       please waiting ... ($g_progress_value%)\n",0,0,2);
	}
	return 0;
	
}

sub get_route_file
{
	my ($route) = @_;
	@_ = ();

	$route =~ s/^(.+)\///g;
	return $route;
	
}


sub code_line_deal
{
	my ($old_file,$new_file) = @_;

	@_ = ();


	my $g_line_id = 1;

	#print("oldfile = $old_file, newfile = $new_file\n");
	open(my $READ_FILE,$old_file)|| die "       can't open $old_file\n";
	open(my $WRITE_FILE,">$new_file") || die "       can't open $new_file\n";
	open(my $TABLE,">>$g_decode_table") || die "       can't open $g_decode_table\n";

	$g_file_name = get_route_file($old_file);

	my $function_change = 0;
	my $function_name = 0;
	my $match_status = 0;
	my $replace_status = 0;
	my @cache_line = ();
	my $montage_line;
	my $new_line;
	my $display_format;


	my $test_en = 0;
	my $hh;
	
	$g_line_id = 1;
	while(my $tmp_line=<$READ_FILE>)
	{
		($function_change,$function_name) = is_function_code_line($tmp_line);
		if($function_change eq 1)
		{
			$g_function_name = $function_name;
			$g_function_id++;
		}
		($match_status, $montage_line) = is_replace_code_line($tmp_line);
		if($match_status eq (-2))
		{
			return (-1);
		}
		elsif($match_status eq (-1))
		{
			while(@cache_line ne 0)
			{
				print $WRITE_FILE shift(@cache_line);
			}
			print $WRITE_FILE $tmp_line;
			$g_line_id++;
		}
		elsif($match_status eq 0)
		{
			push @cache_line,$tmp_line;	
		}
		else
		{
			
			($replace_status,$new_line,$display_format) = dest_replace($montage_line,$match_status,$g_file_id,$g_line_id,$g_function_id);
			#print("return = $replace_status, $new_line, $display_format\n");
			if($replace_status eq 0)
			{
				print_to_screen("       replace error\n",3,2,2);
				print_to_screen("       $montage_line\n",3,2,2);
				return (-1);			
			}
			print $WRITE_FILE "$new_line\n";
			@cache_line = ();
			my $replace_type = 0;
			if($match_status eq 1 or $match_status eq 2)
			{
				$replace_type = 1;
			}
			$display_format =~ s/^"//g;
			$display_format =~ s/"$//g;
			$display_format =~ s/\n//g;
			my $tmp_format = "#fnum<$g_file_id>#file<$g_file_name>#"
				."line<$g_line_id>#func<$g_function_name>#"
				."content<$display_format>#type<$replace_type>#\n";
			print $TABLE $tmp_format;
			$g_line_id++;
		}
	}
	
	if(@cache_line ne 0)
	{
		print_to_screen("       cache_line isn't null at file end\n",3,2,2);
		return (-1);
	}
	
	$g_file_id++;
	close($READ_FILE);
	close($WRITE_FILE);
	close($TABLE);
	return 0;
	
}
