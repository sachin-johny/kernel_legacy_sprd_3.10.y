require "zhuyg.pl";
my $g_function_name;
my $g_final_func_name;
my $g_status = 0;
my $needtoconfirm = 0;
my $g_ifchange = 0;
my $g_lastg_ifchange = 0;
my $print_en = 0;
my $tongji = 0;
####################################################################
#		is_function_code_line start
####################################################################
sub is_function_code_line
{
	my ($string) = @_;
	my $type     = 0;

	if($print_en eq 1)
	{
		print("line = $string\n");
	}
	$string = &mul_line_notes($string,$print_en);						#对多行注释的处理
	if($print_en eq 1)
	{
		print("line out mult = $string\n");
	}
	#$string = &sigle_line_notes($string);						#single
	#my ($change,$func_name) = reconglise_fsm($string);
	#print("change = $change, name = $func_name\n");
	#exit;
	#return ($change,$func_name);

	return &reconglise_fsm($string);#判断语句的语意
	
	#return &judge_block_function($string,$g_status);
}
####################################################################
#
#			语句识别FSM
#g_status:
#    -1:语句
#     1：函数
#     2:#ifdef
#     3:#else 
#     4:#endif
####################################################################   
sub reconglise_fsm
{
	my ($string) = @_;
	my $string_temp = '';
#############################################################################
#line empty
	$string_temp = $string;
	$string_temp =~ s/\s//g;
	if($string_temp !~ /.+/)
	{
		##printf("nothing return\n");
		return (0,'UNKNOW');			#stay the old g_status
	}
##############################################################################
	elsif($string =~ /.*\;.*/)							#排除for结构和一般语句
	{
		if($needtoconfirm == 1)							#not need to change
		{
			$g_ifchange = 0;
			$needtoconfirm = 0;
		}
		$g_status = -1;
	}
	elsif($string_temp =~ /^((while)|(if)|(switch)|(elseif)|(^#)|([a-zA-Z0-9]+\:))/)
	{
		#printf("abc2");
		$tg_status = -1;
	}
	elsif($string_temp =~ /(\')|(\")/)
	{
		#printf("abc2");
		$tg_status = -1;
	}
	elsif($string =~ /\#define/)
	{
		##printf("abc3");
		$g_status = -1;
	}
#*****************************function***************************************
#example:void main()
#############################################################################
	elsif($string =~ /([A-Za-z0-9\_\*]+)\s+([A-Za-z0-9\_\*]+)\s*(\(.*\))/)
	{

		$g_status = 2;
		$needtoconfirm = 1;
		$g_function_name = $2;
		#$g_final_func_name =~ s/\*//;
		#printf("this is a function3\n");
	}
	elsif($string =~ /([A-Za-z0-9\_\*]+)\s+([A-Za-z0-9\_\*]+)\s*(\()([A-Za-z0-9\_\*\,\s]*)/)
	{
		$g_status = 2;
		$needtoconfirm = 1;
		$g_function_name = $2;
		$g_function_name =~ s/\*//;
		##printf("this is a function4\n");
	}
#*****************************function***************************************
#example:void
#	 main()
#############################################################################
	elsif($string =~ /[a-zA-Z0-9\_\*]+\s$/)
	{
		###printf("abc4");
		$g_status = 1;		#maybe function start;
	}
	elsif($string_temp =~ /([A-Za-z0-9\_\*]+)\s*(\(.*\))/)
	{
		if($g_status == 1)
		{
			$g_status = 2;
			###printf("this is a function1\n");
			$needtoconfirm = 1;
			$g_function_name = $1;
		#	$g_final_func_name =~ s/\*//;
		}
	}
	elsif($string_temp =~ /([A-Za-z0-9\_\*]+)\s*(\()([A-Za-z0-9\_\*\,]*)/)
	{
		if($g_status == 1)
		{
			$g_status = 2;
			$needtoconfirm = 1;
			$g_function_name = $1;
			$g_function_name =~ s/\*//;
			###printf("this is a function2\n");
		}
	}
	if($string =~ /.*\{.*/)
	{
		if($needtoconfirm == 1)
		{
			##printf("it will change func\n");
			$g_final_func_name = $g_function_name;				#need to change
			$needtoconfirm = 0;
			$g_ifchange = 1;
			$g_final_func_name =~ s/\*//;
		}
		$g_status = -1;
	}

	if($g_lastg_ifchange == 1)
	{
		$g_lastg_ifchange = 0;
		$g_ifchange = 0;
	}
	if($g_ifchange == 1)
	{
		$g_lastg_ifchange = $g_ifchange;
		$tongji++;
		#printf("tongji".$tongji."\n");
		printf($g_final_func_name."\n");
	}
#################################return#####################################
	#print("$g_ifchange,$g_final_func_name:\t\t$string\n");
	return ($g_ifchange,$g_final_func_name);
}
1;
