#!/usr/bin/perl -w
#use strict;
#use bignum;
my $g_display_timestamp;
my $g_turn_time = 0;

my $g_timestamp;
my $g_timestamp_t = 4;
my $g_pid;
my $g_pid_t = 2;
my $g_log_level;
my $g_log_level_t = 1;
my $g_reserve;
my $g_reserve_t = 1;
my $g_filenum;
my $g_filenum_t = 2;
my $g_linenum;
my $g_linenum_t = 2;
my $g_arg_size;
my $g_arg_size_t = 2;
my $g_add_time = 165;
#my $g_add_time = 4290000000;

my $g_saved_handle;
my $g_output_handler;

my $g_lasttime = 0;
my $g_file_ret_num = 0;
my ($g_log_file)    = "itm_trace.log";
my ($g_format_file) = "decode_table";
my ($g_output_file) = "read_trout_log.txt";
####################################################
#
#	 	    file operation
#
####################################################
sub open_tity_file
{
	my ($filename) = @_;
	my $file_handle;
	open($file_handle,">$filename") || return -1;
	return $file_handle;
}
sub write_tity_file
{
	my ($file_handle,$string) = @_;
	print $file_handle ("$string");
	return 0;
}
#open_file
sub open_file
{
	my ($filename) = @_;
	my $file_handle;
	open($file_handle,"$filename") || return -1;
	return $file_handle;
}
#read_file
sub read_file
{
	my ($file_handle,$skipval,$size) = @_;
	my $result;
#	write_to_file("$size\n");
	$g_file_ret_num = read($file_handle,$result,$size,$skipval);
	return $result;
}
#read_file_bin
sub read_file_bin
{
	my ($file_handle,$skipval,$size) = @_;
	my $result = read_file($file_handle,$skipval,$size);
	return hex(unpack("H*",$result));
}
######################log 格式#############################

############################################################
sub make_time
{
	my ($time_turn,$timestamp) = @_;
	my $temp_time;
	$temp_time = $timestamp % (10**6); #spread num
	$timestamp = $timestamp - $temp_time;
	$timestamp = $timestamp/(10**6) + $time_turn*$g_add_time;									#the sycle is 165s
	$temp_time = sprintf("%06d",$temp_time);
	$temp_time = $timestamp.".".$temp_time;
	#$timestamp = $timestamp + $time_turn*$g_add_time;									#the sycle is 165s
	return $temp_time;
}
sub read_one_log
{
	my ($file_handle_log)=@_;
	my ($temp_timestamp);
	$g_timestamp = read_file_bin($file_handle_log,0,$g_timestamp_t);
	$g_pid       = read_file_bin($file_handle_log,0,$g_pid_t);
	$g_log_level = read_file_bin($file_handle_log,0,$g_log_level_t);
	$g_reserve   = read_file_bin($file_handle_log,0,$g_reserve_t);
	$g_filenum   = read_file_bin($file_handle_log,0,$g_filenum_t);
	$g_linenum   = read_file_bin($file_handle_log,0,$g_linenum_t);
	$g_arg_size  = read_file_bin($file_handle_log,0,$g_arg_size_t);
	$g_saved_handle = $file_handle_log;
#	print("
##		-------------------------------------------------------------------------------------
#		| g_timestamp|    g_pid    | g_log_level |  g_reserve  |   g_filenum  |   g_linenum  | g_arg_size  |
#		|          |           |           |           |            |            |           |
#	--------------------------------------------------------------------------------------
#		|$g_timestamp|     $g_pid    | $g_log_level   \t   |  $g_reserve \t|   $g_filenum\t |   $g_linenum | $g_arg_size
#		|          |           |           |           |            |            |           |
#		--------------------------------------------------------------------------------------
#
#");
	#write_to_file("[cpu =$g_timestamp us] g_filenum:$g_filenum,g_linenum:$g_linenum,arg_size:$g_arg_size\n");	
	if($g_lasttime > $g_timestamp)
	{
		if($g_lasttime > 150000000 and $g_timestamp < 20000000)
		{
			#print("get time skip = $g_timestamp last time = $g_lasttime; new time = $g_display_timestamp <$g_filenum><$g_linenum>\n");
			$g_turn_time++;
		}
	}

	$g_display_timestamp = make_time($g_turn_time,$g_timestamp);
	if($g_lasttime > $g_timestamp and $g_timestamp != 0)
	{
		print("get time skip = $g_timestamp last time = $g_lasttime; new time = $g_display_timestamp <$g_filenum><$g_linenum>\n");
	}
	if($g_file_ret_num == 0)
	{
#		printf("end of file\n");
		return -1;
	}
	if($g_timestamp != 0 && $g_pid == 0 && $g_log_level == 0 && $g_reserve == 0 && $g_filenum == 0 && $g_linenum == 0)# && $g_arg_size == 0)
	{	
		write_to_file("[$g_display_timestamp] long time no log export! struct len = $g_arg_size\n");
		return 2;
	}
	$g_lasttime  = $g_timestamp;
	return 0;
}
###########################write_to_file###############################
sub write_to_file
{
	my ($combine) = @_;
	return write_tity_file($g_output_handler,$combine);
}
##############################put into##################################
sub put_arg_into_format
{
	my ($content) =@_;
	my $i=0;
	my @temp_array;
	my $new_string = '';
	my $read_from_file;
	my $temp_1;
	my $temp_2;
	my $temp_3;
	my $temp_4 = 0;
	my $string;
	my $format;
	if($g_arg_size == 0)
	{
		#printf("no args\n");
		return $content;
	}
#	write_to_file("the size is:$g_arg_size,$content\n");
	@temp_array = split('%',$content);
	while($i< @temp_array)
	{
		if($i == 0)
		{
			$new_string .= "$temp_array[$i]";
			$i++;
			next;
		}
		if($temp_array[$i] =~ /^([0-9\.]*)([pdu])/)
		{
			$temp_1 = $1;
			$temp_2 = $2;
			$format  = '%'.$temp_1.$temp_2;
			$read_from_file = read_file_bin($g_saved_handle,0,4);
			$read_from_file = sprintf($format,$read_from_file);
			$temp_array[$i] =~ s/^([0-9\.]*)([pdu])/$read_from_file/;
			$temp_2 = 0;
			$temp_2 = 0;
			$g_arg_size -= 4;
			$temp_4 = 1;			#for_check_in
			#write_to_file("in pdu\n");
		}
		elsif($temp_array[$i] =~ /^s/)
		{
			do
			{
				$temp_3 = read_file($g_saved_handle,0,1);
				if($temp_3 ne "\0")
				{
					$string .= $temp_3;
				}
				$g_arg_size -= 1;
			}while($temp_3 ne "\0");
			$temp_array[$i] =~ s/^s/$string/;
			$string = '';
			$temp_3 = 0;
			$temp_4 = 1;			#for_check_in
#			write_to_file("in %%s\n");
 		}
		elsif($temp_array[$i] =~ /^c/)
		{
			$read_from_file = read_file($g_saved_handle,0,1);
			$temp_array[$i] =~ s/^c/$read_from_file/;
			$g_arg_size -= 1;
			$temp_4 = 1;			#for_check_in
			#write_to_file("in %%c\n");
		}
		elsif($temp_array[$i] =~ /^([0-9\#]*)([xX])/o)
		{
			$temp_1 = $1;
			$temp_2 = $2;
			$format = '%'.$temp_1.$temp_2;
			$read_from_file = read_file_bin($g_saved_handle,0,4);
			$read_from_file = sprintf($format,$read_from_file);
			$temp_array[$i] =~ s/^([0-9\#]*)([xX])/$read_from_file/;
			$temp_1 = 0;
			$temp_2 = 0;
			$g_arg_size -= 4;
			$temp_4 = 1;			#for_check_in
			#write_to_file("in %%x\n");
		}
		$new_string .= "$temp_array[$i]";
		if($i != 0)
		{
			if($temp_4 != 1)
			{
				printf("$i:%% in the clouse have not any type so it hadn't handled:\t$temp_array[$i]\n");
				$temp_4  = 0;
			}
		}
		$i++;
	}
	if($g_arg_size != 0)
	{
		printf("there are some error that the size is $g_arg_size\n");
		read_file($g_saved_handle,0,$g_arg_size);	#adjust the file pointor to the next log;
	}

	return $new_string;
}
##########################combine_type_3################################
sub combine_type_3
{
	my ($format) = @_;
	my $file_name;
	my $function_name = 0;
	my $content = 0;
	my $combine;
	my $index;
	if($format =~ /(#file<)(.*\.((cpp)|(c)|(h)))(>)/)
	{
		$file_name = $2;
		#printf("$file_name\n");
	}
	if($format =~ /(#func<)([A-Za-z0-9\_]+)(>)/)
	{
		$function_name = $2;
		#printf("$function_name\n");
	}
	if($format =~ /(content<)(.+)(>#type)/)
	{
		$content = $2;
	}
	###################################
	$content = put_arg_into_format($content);
	###############combine#############
	if($display_config[0] == 1)
	{
		$combine = "[$g_display_timestamp] " ;
	}
	if($display_config[1] == 1)
	{
		$combine .= "$file_name " ;
	}
	if($display_config[2] == 1)
	{
		$combine .= "$function_name " ;
	}
	if($display_config[3] == 1)
	{
		$combine .= "$g_linenum " ;
	}
	if($display_config[4] == 1)
	{
		$combine .= "$g_pid " ;
	}	
	if($display_config[5] == 1)
	{
		$combine .= "$g_log_level " ;
	}	
	if($display_config[6] == 1)
	{
		$combine .= "$content\n" ;
	}
	write_to_file($combine);
}
########################combine_type_1_2################################
sub combine_type_1_2
{
	my ($format,$type) = @_;
	my $file_name;
	my $function_name;
	my $content;
	my $combine;
#	printf("fdsfdfdfdfdf");
	if($format =~ /(#file<)(.*\.((cpp)|(c)|(h)))(>)/)
	{
		$file_name = $2;
	#	printf("$file_name\n");
	}
	if($format =~ /(#func<)([A-Za-z0-9\_]+)(>)/)
	{
		$function_name = $2;
	#	printf("$function_name\n");
	}
	##################combine###################
	if($display_config[0] == 1)
	{
		$combine = "[$g_display_timestamp] " ;
	}
	if($display_config[1] == 1)
	{
		$combine .= "$file_name " ;
	}
	if($display_config[2] == 1)
	{
		$combine .= "$function_name " ;
	}
	if($display_config[3] == 1)
	{
		$combine .= "$g_linenum " ;
	}
	if($display_config[4] == 1)
	{
		$combine .= "$g_pid " ;
	}	
	if($display_config[5] == 1)
	{
		$combine .= "$g_log_level " ;
	}	
	if($display_config[6] == 1)
	{
		$combine .= "$function_name $type\n" ;
	}
	#printf("the size is",$g_arg_size."\n");
	read_file($g_saved_handle,0,$g_arg_size);    #ajust
	write_to_file($combine);
}
#######################get_the_format_type##############################
sub get_the_format_type
{
	my ($format) = @_;
	my $type = 0;
	if($format =~ /.*+/)
	{
	}
	else
	{
		return -1;
	}
	if($format =~ /\#type<[1]>/)
	{
	#	printf("goto the type1\n");
		if($format =~ /(enter)/i)
		{
	#		printf("goto the type enter\n");
			combine_type_1_2($format,"enter");
		}
		elsif($format =~ /(exit)/i)
		{
		#	printf("goto the type exit\n");
			combine_type_1_2($format,"exit");
		}
	}
	elsif($format =~ /\#type\<0\>/)
	{
		combine_type_3($format);
	}
}
#####################get corrosponding format###########################
sub read_one_log_format
{
	my ($array) = @_;
	my $index;
	my $format = '';
	$index = $g_filenum."\_".$g_linenum;
	$format = $array->{$index};
	if($format ne '')
	{
	#	printf("format found ok:$format\n");
		get_the_format_type($format);
	}
	else
	{
		return -1;
	}
}
sub build_the_array
{
	my ($array,$file_handle) = @_;
	my $f_num;
	my $l_num;
	my $index = '';
	while($format = <$file_handle>)
	{
		if($format =~ /#fnum<([0-9]+)>#file.*/)
		{
			$f_num = $1;
		}
		if($format =~ /#line<([0-9]+)>#func.*/)
		{
			$l_num = $1;
		}
		#printf($f_num."\t".$l_num."\n");
		$index = $f_num."\_".$l_num;
		#printf("index is:".$index."\n");
		if($index ne '')
		{
			$array->{$index} = $format;
		}
	}
	return $array;
}
######################################################
sub handle_log
{
	my $result;
	my $file_handle_log;
	my $file_handle;
	my %array;
	$file_handle_log = open_file($g_log_file);				#log file
	if($file_handle_log == -1)
	{
		printf("[error]file $g_log_file open error\n");
		return -1;
	}
	binmode($file_handle_log);
	$file_handle = open_file($g_format_file);				#format file
	if($file_handle == -1)
	{
		printf("[error]file $g_format_file open error\n");
		return -1;
	}
	$g_output_handler =  open_tity_file($g_output_file);
	if($g_output_handler == -1)
	{
		printf("[error]file $g_output_file open error\n");
		return -1;
	}
	$array = build_the_array($array,$file_handle);							#build the fomat array
	while(1)	
	{
		$result = read_one_log($file_handle_log);			#get the log
		if($result == -1)								#end of file
		{
			return 0;
		}
		elsif($result == 2)
		{
			next;
		}
		#seek $file_handle,0,0;
		#read trout_log ok and now read trout_code_lut
		$result = read_one_log_format($array);		#combine log and the format
		if($result == -1)
		{
			printf("[error]these is no format meet the file:file_num:$g_filenum,line_num:$g_linenum\n");
			return -1;
		}
	}
	close($g_output_handler);
	close($file_handle);								#close format handle
	close($file_handle_log);							#close log handle
}
####################################################
#
#	 	    main operation
#
####################################################
sub log_restore
{
	local (@display_config) = @_;
	return handle_log();
}
