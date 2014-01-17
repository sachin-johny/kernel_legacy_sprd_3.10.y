use Spreadsheet::WriteExcel;
use strict;

my %g_content = ();
my %g_col_width = ();
my $g_line_num = 0;

sub get_content
{
	my ($line_content) = @_;
	@_ = ();
	my ($i) = 0;
	my ($tmp) = 0;
	my ($num) = 0;

	$num = scalar(@$line_content);
	$g_content{$g_line_num}->{'word_num'} = $num;

	for ($i = 0; $i < $num; $i++)
	{
		$tmp = 0;
		$g_content{$g_line_num}->{$i} = $$line_content[$i];

		$tmp = rindex $$line_content[$i]."\$", "\$";
		#print("col is: $i, len is: $tmp\n");

		if ($tmp > 8)
		{ 
			$tmp = $tmp + 3;
			if ($tmp > $g_col_width{$i})
			{
				$g_col_width{$i} = $tmp;
			}
		}
		else
		{
			if (defined $g_col_width{$i})
			{
				next;
			}
			else
			{
				$g_col_width{$i} = 8.53;
			}
		}
	}	

	$g_line_num++;
}

sub create_new_file
{
	my ($f_name) = @_;
	@_ = ();
	$f_name = $f_name;
	
	my ($row) = 0;
	my ($col) = 0;
	my ($words) = 0;
	my ($value) = 0;

	my $workbook = 0; 
	my $worksheet = 0;
	my $format = 0;

	$workbook =  Spreadsheet::WriteExcel->new($f_name);
	$worksheet = $workbook->add_worksheet();

	$format = $workbook->add_format();
	$format->set_bold();
	$format->set_color('black');
	$format->set_align('center');

	for ($row = 0; $row < $g_line_num; $row++)
	{
		$words = $g_content{$row}->{'word_num'};
		for ($col = 0; $col < $words; $col++)
		{
			#print ("col is: $col, set width is: $g_col_width{$col}\n");
 			$worksheet->set_column($col, $col, $g_col_width{$col});
			$value = $g_content{$row}->{$col};
			$worksheet->write($row, $col, $value, $format);
		}
		#print ("---------------\n");
	}

	$workbook->close();
}

1;
