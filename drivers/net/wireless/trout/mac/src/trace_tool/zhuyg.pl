#!/usr/bin/perl

my @g_replace_code = ("TROUT_FUNC_ENTER", 
		 	"TROUT_FUNC_EXIT", 
			"TROUT_DBG()",
			"TROUT_TX_DBG()",
                     "TROUT_RX_DBG()",
                     "TROUT_INT_DBG()",
		     "TROUT_PRINT()"); 		#need main give
my @g_display_content = ("%s: enter", "%s: exit"); 	#need main give
my @g_replace_function = ("trout_trace_print_log");     #need main give
my $default_debug_level = 4; 				#need main give
my $g_picture_save_path = 'result/';    			#need main give

my $g_new_line = ""; 		#for function is_replace_code_line
my $g_replace_index = 0;  	#for function is_replace_code_line
my $g_right_bracket_count = 0; 	#for function is_replace_code_line
my $g_left_bracket_count = 0; 	#for function is_replace_code_line
my $g_continue_flag = 0; 	#for function is_replace_code_line
my $g_mul_note = 0; 		#for function mul_line_notes

sub get_format;
sub get_behind_num;
sub mul_line_notes;
sub sigle_line_notes;
sub is_good_line_code;
sub get_str_in_bracket;
sub get_str_in_quotation;
sub remove_content_in_quotation;

#******************************************************************** 
#*
#* Function Name : is_replace_code_line
#*
#* Description   : check weather the code line is our replace code
#*
#* Globals       :replace_code. ----the code need to be replaced 
#*               :new_line -----the line content after merge
#*               :replace_index ---the code need to be replaced index
#*               :right_brack_count ---  right bracket count
#*               :left_brack_count ---  left bracket count
#*               :continue_flag --- weather need more info to merge the replace code.
#*
#* Inputs        : code line content
#*
#* Outputs       : None
#*
#* Returns       : (-2, new_line) --- multiple statement in one line.
#*	 	   (-1, new_line)--- not our replace code.  new_line content
#*                 (0, new_line) -is our replace code line. need more information to merge the code line.
#*                 (replace_index, new_line) --- is our replace code line, replace index
#*
#********************************************************************
sub is_replace_code_line
{
	my ($line_content) = @_;
	@_ = ();
	my ($count) = 0;
	my ($tmp) = 0;
	my ($bracket_flag) = 0;
	my (@tmp_array) = ();
	
	$count = @g_replace_code;

	#filter space, tab and return 
	$line_content =~ s/^ +//;
	$line_content =~ s/^\t+//;
	chomp($line_content);

	$line_content = &mul_line_notes($line_content); 	
	$line_content = &sigle_line_notes($line_content); 

	if ($g_continue_flag == 1)
	{
		#use "()" num to check weather the code line is over
		$g_new_line = $g_new_line.$line_content;
		while ($line_content =~ /\(/g)
		{
			$g_left_bracket_count++;
		}
		while ($line_content =~ /\)/g)
		{
			$g_right_bracket_count++;
		}
		if ($g_left_bracket_count - $g_right_bracket_count <= 0)
		{
			$g_continue_flag = 0;
			$g_left_bracket_count = 0;
			$g_right_bracket_count = 0;
			return ($g_replace_index+1, $g_new_line);
		}
		else
		{
			$g_continue_flag = 1;
			return (0, $g_new_line);
		}
	}

	$g_replace_index = 0;

	while ($g_replace_index < $count)
	{
		$tmp = $g_replace_code[$g_replace_index];

		#get the source code before "("
		if ($tmp =~ /\(/)
		{
			$bracket_flag = 1;
			@tmp_array = split(/\(/, $tmp);
			$tmp = $tmp_array[0];	
		}

		if ($line_content =~ m/\Q$tmp\E[0-9]*/)
		{
			$g_new_line = $line_content;
			$tmp = &is_good_line_code($line_content);
			if ($tmp == 0)
			{
				print("multiple statement in one line\n");
				return (-2, '');
			}
		
			if ($line_content =~ /^#define/)
			{
				return (-1, '');
			}
			if ($bracket_flag == 1)
			{
				while ($line_content =~ /\(/g)
				{
					$g_left_bracket_count++;
				}
				while ($line_content =~ /\)/g)
				{
					$g_right_bracket_count++;
				}
				if ($g_left_bracket_count - $g_right_bracket_count <= 0)
				{
					if ($g_left_bracket_count > 0)
					{
						last;
					}
				}
				else
				{
					$g_continue_flag = 1;
					return (0, $line_content);
				}
			}
			else
			{
				last;
			}
			
		}

		$g_replace_index = $g_replace_index + 1;
	}
	if ($g_replace_index == $count)
	{
		$g_replace_index = -2;
	}

	$g_right_bracket_count = 0;
	$g_left_bracket_count = 0;
	$g_continue_flag = 0;

	return($g_replace_index+1, $g_new_line);
}

#******************************************************************** 
#*
#* Function Name : dest_replace
#*
#* Description   : replace dest code
#*
#* Globals       : (@display_conentet)---the display content of type
#*		   ($replae_function)---replace's function
#*
#* Inputs        : ($line_content)---line content
#*		  ($replace_index)---the index need to be replaced
#*		  ($file_num)
#*		  ($line_num)
#*		  ($function_name_num)
#*
#*
#* Outputs       : None
#*
#* Returns       : ($ret)  --- 0 replace fail. 		1 replace ok
#*                 ($new_line_content) --- after replaed line
#*                 ($display) --- the str in quotation
#*
#********************************************************************

sub dest_replace
{
	my ($line_content, $replace_index,
		 $file_num, $line_num, $function_name_num) = @_;
	my ($debug_num) = -1;
	my ($line_tmp) = '';
	my ($display) = '';
	my ($new_line_content) = '';
	my ($ret) = 1;
	my ($format) = '';
	my ($arg) = '';
	my ($num) = 0;

	$num = @g_replace_code;


	$line_content =~ s/\s+//;
	$debug_num = &get_behind_num($line_content);
	if ($debug_num eq -1)
	{
		$debug_num = $default_debug_level;
	}
	$display = $g_display_content[$replace_index-1];
	if ($replace_index gt $num)
	{
		print ("type can not find: $num\n");
		print ("line content is: $line_content\n");
		$ret = 0;
	}
	elsif($replace_index eq 1)
	{
		$line_tmp = "($debug_num, $file_num, $line_num, \"d\", $function_name_num);";
	}
	elsif ($replace_index eq 2)
	{
		$line_tmp = "($debug_num, $file_num, $line_num, \"d\", $function_name_num);";
		
	}
	else
	{
		$line_content = &get_str_in_bracket($line_content);
		($format, $arg) = &get_str_in_quotation($line_content);
		$display = $format;
		$format = &get_format($format);
		$line_tmp = "($debug_num, $file_num, $line_num, \"$format\" $arg);";
		
	}
	
	$new_line_content = $g_replace_function[0].$line_tmp;
	
	return ($ret, $new_line_content, $display);
}

#********************************************************************
#*
#* Function Name : line_grap
#*
#* Description   : draw a line graph for input data
#*
#* Globals       : $picture_save_pat --- the picture will be saved path
#*
#* Inputs        : ($type) --- what type will be graph
#*               : ($xdata)--- the pointer to x coordniate data
#*               : ($ydata)--- the pointer to y corrdniate data
#*               : ($x_name)--- x coordniate name
#*               : ($y_name)----y coordniate name
#*
#*Outputs       : a picture about graph
#*
#*Returns:         (0) --- fail
#*                 (1)  ---success.
#*
#********************************************************************
sub line_grap
{
	use strict;
	use perlchartdir;

	my ($type, $xdata, $ydata, $x_name, $y_name) = @_;
	@_ = ();

	my ($picture_name) = '';
	my ($date) = '';
	my ($setp) = 1;
	my ($count) = 0;
	my ($max_point) = 24;
	$count = scalar(@$xdata);

	if ($count gt $max_point)
	{
		$setp = $count / $max_point;
		if ($setp =~ /\./)
		{
			$setp =~ s/\..*//;
			$setp++;
		}	
	}
	$picture_name = $type.'.jpg';	
	$picture_name = $g_picture_save_path.$picture_name;	

	my $c = new XYChart(1300, 800, 0xffffc0, 0x000000, 1);
	$c->setPlotArea(100, 100, 1100, 600, 0xffffff, -1, -1, 0xc0c0c0, -1);
	# Add a line chart layer using the given data
	$c->addLineLayer($ydata);
	# Set the labels on the x axis.
	$c->xAxis()->setLabels($xdata);
	# Display 1 out of 3 labels on the x-axis.
	$c->xAxis()->setLabelStep($setp);
	$c->xAxis()->setTitle("X: $x_name");
	$c->yAxis()->setTitle("Y: $y_name");
	$c->addTitle("$type");
	# output the chart
	$c->makeChart("$picture_name");
}

#********************************************************************
#*
#* Function Name : pie_grap
#*
#* Description   : draw a pie graph for input data
#*
#* Globals       : $picture_save_path --- the picture will be saved path
#*
#* Inputs        : ($type) --- what type will be graph
#*               : ($data)--- the pointer to datas array
#*               : ($labels)--- the pointer to labels array
#*
#*Outputs       : a picture about graph
#*
#*Returns:         (0) --- fail
#*                 (1)  ---success.
#*
#********************************************************************
sub pie_grap
{
	use strict;
	use perlchartdir;

	my ($type, $data, $labels) = @_;
	@_ = ();

	my ($picture_name) = '';
	my ($date) = '';
	my ($count) = 0;
	my ($radius) = 0;
	my ($length) = 1360;
	my ($width) = 0;
	$count = scalar(@$data);

	#$date = &get_time();
	$picture_name = $type.'.png';	
	$picture_name = $g_picture_save_path.$picture_name;	

	if ($count > 0 and $count <= 30)
	{
		$width = 500;
		$radius = 200;
	}
	elsif ($count > 30 and $count <= 60)
	{
		$width = 700;
		$radius = 250;
	}
	elsif ($count > 60 and $count <= 100)
	{
		$length = 1800;
		$width = 1200;
		$radius = 300;
	}
	else
	{
		print ("point is $count, too many, cannot draw\n");
		return -1;
	}
	# Create a PieChart object of size 560 x 270 pixels, with a golden background and a 1
	# pixel 3D border
	my $c = new PieChart($length, $width, perlchartdir::goldColor(), -1, 1);
	
	# Add a title box using 15 pts Times Bold Italic font and metallic pink background
	# color
	$c->addTitle($type, "timesbi.ttf", 15)->setBackground(
	    perlchartdir::metalColor(0xff9999));
	
	# Set the center of the pie at (280, 135) and the radius to 110 pixels
	$c->setPieSize($length/2, $width/2, $radius);
	
	# Draw the pie in 3D with 20 pixels 3D depth
	$c->set3D(20);
	
	# Use the side label layout method
	$c->setLabelLayout($perlchartdir::SideLayout);
	
	# Set the label box background color the same as the sector color, with glass effect,
	# and with 5 pixels rounded corners
	my $t = $c->setLabelStyle();
	$t->setBackground($perlchartdir::SameAsMainColor, $perlchartdir::Transparent,
	    perlchartdir::glassEffect());
	$t->setRoundedCorners(5);
	
	# Set the border color of the sector the same color as the fill color. Set the line
	# color of the join line to black (0x0)
	$c->setLineColor($perlchartdir::SameAsMainColor, 0x000000);
	
	# Set the start angle to 135 degrees may improve layout when there are many small
	# sectors at the end of the data array (that is, data sorted in descending order). It
	# is because this makes the small sectors position near the horizontal axis, where
	# the text label has the least tendency to overlap. For data sorted in ascending
	# order, a start angle of 45 degrees can be used instead.
	$c->setStartAngle(135);
	
	# Set the pie data and the pie labels
	$c->setData($data, $labels);
	
	# Output the chart
	$c->makeChart($picture_name);

	return 1;
}
#******************************************************************** 
#*
#* Function Name : get_behind_num
#*
#* Description   : get the number behind str
#*
#* Globals       : None
#*
#* Inputs        : ($line_content)---line content
#*
#* Outputs       : None
#*
#* Returns       : (-1) fail
#*                 (n)--the number
#*
#********************************************************************
sub get_behind_num
{
	my ($line_content) = @_;
	
	if ($line_content =~ /\w+\s*/)
	{
		while ($& =~ /[a-zA-Z\_]+/g)
		{
			$ret = $';
		}
	}

	if ($ret =~ /[0-9]+/)
	{
		return $ret;
	}

	
	return -1;
}

#******************************************************************** 
#*
#* Function Name : get_str_in_bracket
#*
#* Description   : get str within bracket
#*
#* Globals       : None
#*
#* Inputs        : ($line_content)---line content
#*
#* Outputs       : None
#*
#* Returns       : ('')-- fail
#*                 ($ret_str)--the str within bracket
#*
#********************************************************************
sub get_str_in_bracket
{
	my ($line_content) = @_;
	my ($ret_str) = '';
	my ($right_bracket_count) = 0;
	my ($left_bracket_count) = 0;
	
	while ($line_content =~ /\(/g)
	{
		$left_bracket_count++;
	}
	
	while ($line_content =~ /\)/g)
	{
		$right_bracket_count++;
		if ($left_bracket_count - $right_bracket_count <= 0)
		{
			if ($` =~ /(\()/)
			{
				$ret_str = $';
			}
			last;
		}
	}

	return $ret_str;
}

#******************************************************************** 
#*
#* Function Name : get_str_in_quotation
#*
#* Description   : get str within quotation
#*
#* Globals       : None
#*
#* Inputs        : ($line_content)---line content
#*
#* Outputs       : None
#*
#* Returns       : ('')-- fail
#*                 ($inside_str)--the str within quotation
#*                 ($outside_str)--the str outside quotation
#*
#********************************************************************
sub get_str_in_quotation
{
	my ($line_content) = @_;
	@_ = (); 
	my ($inside_str) = '';
	my ($outside_str) = '';
	my ($count ) = 0;
	
	$line_content =~ s/\s+//;
	while ($line_content =~ /"/g)
	{
		unless ($` =~ /\\$/)
		{
			$count++;
			if ($count eq 2)
			{
				$inside_str = $`.$&;
				$outside_str = $';
				if ($inside_str =~ /^[a-zA-Z0-9]+/)
				{
					if ($inside_str =~ /"/)
					{
						$inside_str = $&.$';
					}
				}
				last;
			}
		}
	}


	return ($inside_str, $outside_str);
}

#******************************************************************** 
#*
#* Function Name : get_format
#*
#* Description   : get format char
#*
#* Globals       : None
#*
#* Inputs        : ($line_content)---line content
#*
#* Outputs       : None
#*
#* Returns       : ('')-- fail
#*                 ($format_list)--the list of format char
#*
#********************************************************************
sub get_format
{
	my ($line_content) = @_;
	@_ = (); 

	my ($format_list) = '';
	my ($tmp) = 0;
	my ($i) = 0;
	my ($count) = 0;
	my ($tmp_array) = '';
	my (@format_array) = ('i', 'd', 'o', 'x', 'X','p', 'c', 'f', 'F', 'e', 'E', 's', 'u', 'G', 'g', 'a', 'A', 'n');
	
	@tmp_array = split(/%/, $line_content);
	$count = @tmp_array;
	
	for ($i = 1; $i < $count; $i++)
	{
		if ($tmp_array[$i] =~ /[a-zA-Z]/)
		{
			$tmp_array[$i] = $&;
		}

		foreach $tmp(@format_array)
		{
			if ($tmp_array[$i] eq $tmp)
			{
				$format_list=$format_list.$tmp;
				last;
			}
		}
	}

	return $format_list;
}

####################################################################
#
#			单行注释判断
#
#################################################################### 
sub sigle_line_notes
{
	my ($string) = @_;
	my $temp_1;
	if($string =~ /(.*)(\/\/)(.*)/)
	{
		$temp_1 = $1;
		if($g_mul_note > 0)	#check if there is some error judge!
		{
			#$g_mul_note--;
			return '';
		}
		#elsif($g_mul_note < 0)
		#{
		#	$g_mul_note++;
		#}
		if($1 !~ /[a-zA-Z0-9\{\}]+/)
		{
			return '';
		}
		else
		{
			$string = $temp_1;
		}
	}
	return $string;
}

####################################################################
#
#			多行注释判断
#
#################################################################### 
sub mul_line_notes
{
	my ($string) = @_;
	my $temp_1;
	my $temp_2;  

	$string =~ s/\/\*.*\*\\//g;
	while ($string =~ /(.*)\/\//)
	{
		$string = $1;
	}
	if($string =~ /(.*)(\/\*.*\*\/)(.*)/)
	{
		$temp_1 = $1;
		$temp_3 = $3;
		#解决注释左右两边的有效数据
		if($1 !~ /[a-zA-Z0-9\{\}]+/ && $3 !~ /[a-zA-Z0-9\{\}]+/)		
		{
			return '';
		}
		else
		{
			$string = $temp_1.$temp_3;
		}
	}
	elsif($string =~ /(.*)(\/\*)/)
	{
		$temp_1 = $1;
		#解决注释左边的有效数据
		if($1 !~ /[a-zA-Z0-9\{\}]+/)			
		{
			$g_mul_note++;
			return '';
		}
		else
		{
			$string = $temp_1;
			$g_mul_note++;
		}
	}
	elsif($string =~ /(\*\/)(.*)/)
	{
		$temp_1 = $2;
		#解决注释右边的有效数据
		if($2 !~ /[a-zA-Z0-9\{\}]+/)
		{
			$g_mul_note--;
			return '';
		}
		else
		{
			$g_mul_note--;
			$string = $temp_1;
		}
	}
	elsif($g_mul_note > 0)							#处于注释中的语句块
	{
		return '';
	}

	while ($string =~ /\/\*.*\*\//)
	{
		$string = &mul_line_notes($string);
	}
	return $string;
}

sub remove_content_in_quotation
{
	my ($line_content) = @_;
	@_ = ();

	my ($count) = 0;
	my ($tmp1) = $line_content;
	
	while ($line_content =~ /"/g)
	{
		unless ($` =~ /\\$/)
		{
			$count++;
			if ($count eq 1)
			{
				$tmp1 = $`;	
			}
			elsif($count eq 2)
			{
				$tmp1 = $tmp1.$';
			}
		}
	}

	if ($tmp1 =~ /".*"/)
	{
		$tmp1 = &remove_content_in_quotation($tmp1);
	}
	
	return $tmp1
}
#******************************************************************** 
#*
#* Function Name : is_good_line_code
#*
#* Description   : check code line is standard format
#*
#* Globals       : None
#*
#* Inputs        : code line content
#*
#* Outputs       : None
#*
#*Returns:	   (0) --- bad line. multipy statement in one line
#*      	   (1)  --- good line .
#*
#********************************************************************
sub is_good_line_code
{
	my($line_content) = @_;
	@_ = ();
	my ($count) = 0;

	$line_content = &remove_content_in_quotation($line_content);

	while ($line_content =~ /;/g)
	{
		$count++;
	}
	
	
	if ($count > 1)
	{
		if ($line_content =~ /for/)
		{
			if ($count > 3)
			{
				print("multipy statement in one line\n");
				return 0;
			}
		}
		else
		{
			print("multipy statement in one line\n");
			return 0;
		}
		
	}
	
	return 1;
}



1;
