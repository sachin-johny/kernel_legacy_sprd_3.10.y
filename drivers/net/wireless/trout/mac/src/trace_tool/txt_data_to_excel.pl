use strict;
use warnings;
use Spreadsheet::WriteExcel;


# globle variable
my @g_workbook = {};
my @g_format = {};

my @g_excel_line = (10000, 10000, 10000, 10000);

my $g_regrd_value = 0;
my $g_regwt_value = 1;
my $g_ramrd_value = 2;
my $g_ramwt_value = 3;

my @g_sheet = (0, 0, 0, 0);
my @g_row = (0,0,0,0);
my @g_title = (0,0,0,0);

my @g_excel_file = ('reg_read.xls', 'reg_write.xls', 'ram_read.xls', 'ram_write.xls');

my @g_regrd_title = ('address', 0, 17, 13, 9, 1, 2, 3, 4, 5, 6, 7);
my @g_regwt_title = ('address', 0, 17, 13, 9, 1, 2, 3, 4, 5, 6, 7);
my @g_ramrd_title = ('address', 'length', 0, 1, 2, 3, 5, 2, 4, 5, 6, 1, 2, 3, 5, 2, 4, 5, 6, 7);
my @g_ramwt_title = ('address', 'length', 0, 1, 2, 3, 5, 2, 4, 5, 6, 1, 2, 3, 5, 2, 4, 5, 6, 7);

my $g_txt_file = 'trout_io.log';
my $g_line_num = 0;
my $G_FILE;
my $g_id = 0;

main();

sub main
{
	open($G_FILE, $g_txt_file) || die "\nCan't open file $g_txt_file\n\n";
	create_excel_file();
	write_excel_title();
	while(my $line = <$G_FILE>)
	{
		$g_line_num++;
		print "line:$g_line_num\n";
		$line =~ s/\n//g;
		$g_id = 0;
		if($line =~ /regrd/)
		{
			$g_id = 0;
		}
		elsif($line =~ /regwt/)
		{
			$g_id = 1;
		}
		elsif($line =~ /ramrd/)
		{
			$g_id = 2;
		}
		elsif($line =~ /ramwt/)
		{
			$g_id = 3;
		}
		else
		{
			print "\nfind invalid line $g_line_num <$line>\n\n";
			next;
		}
		if($g_excel_line[$g_id] > 0)
		{
			$g_excel_line[$g_id]--;
		}
		else
		{
			check_end();
			next;
		}
		my @content = get_line_data($line);
		my $content_num = @content;
		write_excel($g_id, \@content, $content_num);
	}
	
	close_all();
}


sub check_end
{
	for(my $i = 0; $i < 4; $i++)
	{
		if($g_excel_line[$i] > 0)
		{
			return;
		}
	}
	close_all();
}


sub get_line_data
{
	my ($line) = @_;
	@_ = ();

	my @sub = split(/\|/, $line);
	my $num = @sub;
	my @result = ();
	my $idx = 0;
	for(my $i = 0; $i < $num; $i++)
	{
		# get addr
		if($i == 0)
		{
			if($sub[$i] =~ /(0x[0123456789abcdef]+) - (\d+) /)
			{
				push @result, $1;
				push @result, $2;
				$idx = 2;
			}
			elsif($sub[$i] =~ /(0x[0123456789abcdef]+) /)
			{
				push @result, $1;
				$idx = 1;
			}
			else
			{
				print "\nFind format <$sub[$i]> error line $g_line_num <$line>\n\n";
				close_all();
			}
		}
		# get data
		if($sub[$i] =~ /(\d+):(\d+)/)
		{
			my $sign = $1;
			my $data = $2;
			#print "test $sign $data\n";
			my $sign_array;
			
			if($g_id == 0) { $sign_array = \@g_regrd_title; }
			elsif($g_id == 1) { $sign_array = \@g_regwt_title; }
			elsif($g_id == 2) { $sign_array = \@g_ramrd_title; }
			elsif($g_id == 3) { $sign_array = \@g_ramwt_title; }
			
			my $sign_num = scalar(@$sign_array);
			
			for(my $j = $i+$idx; $j<$sign_num; $j++)
			{
				#print "sign_array [$j] = @$sign_array[$j]\n";
				if($sign == @$sign_array[$j])
				{
					push @result, $2;
					last;
				}
				else
				{
					push @result, '';
					$idx++;
				}
			}
		}
		else
		{
			print "\nFind sub format $i error line $g_line_num <$line>\n\n";
			close_all();
		}
	}
	my $res_num = @result;
	if($res_num < 5)
	{
		print "\nCan't file match sign $g_id\n";
		print "\nline $g_line_num <$line>\n\n";
		close_all();
	}
	return @result;
}

sub create_excel_file
{
	for(my $i = 0; $i < 4; $i++)
	{
		if(-d $g_excel_file[$i])
		{
			system("rm -rf $g_excel_file[$i]");
		}

		$g_workbook[$i] = Spreadsheet::WriteExcel->new($g_excel_file[$i]);
		$g_sheet[$i] = $g_workbook[$i]->add_worksheet();
		$g_format[$i] = $g_workbook[$i]->add_format();
		$g_format[$i]->set_bold();
		$g_format[$i]->set_color('black');
		$g_format[$i]->set_align('center');
	}

}


sub close_excel_file
{
	for(my $i = 0; $i < 4; $i++)
	{
		$g_workbook[$i]->close();
	}

}


sub write_excel_title
{
	my $idx = 0;
	
	$idx = @g_regrd_title;
	write_excel(0, \@g_regrd_title, $idx);

	$idx = @g_regwt_title;
	write_excel(1, \@g_regwt_title, $idx);

	$idx = @g_ramrd_title;
	write_excel(2, \@g_ramrd_title, $idx);

	$idx = @g_ramwt_title;
	write_excel(3, \@g_ramwt_title, $idx);

}

sub write_excel
{
	my ($sheet_num, $content, $cell_num) = @_;
	@_ = ();

	my @col = @$content;
	#print "<@col>\n";
	for(my $i = 0; $i < $cell_num; $i++)
	{
		$g_sheet[$sheet_num]->write($g_row[$sheet_num], $i, $col[$i],$g_format[$sheet_num]);
	}
	$g_row[$sheet_num]++;
}


sub close_all
{
	close_excel_file();
	close($G_FILE);
	exit;
}

