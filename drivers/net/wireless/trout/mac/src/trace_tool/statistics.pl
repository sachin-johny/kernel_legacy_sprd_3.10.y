use strict;
use warnings;

our %g_statistics = ();
my $g_log_file = 'read_trout_log.txt';

require 'init_formula.pl';
require 'execute_formula.pl';
require 'zhuyg.pl';


sub statistics
{
	# initial statistics formula
	my $return = statistics_init();
	if($return eq (-1))
	{
		print("find statistics formula error!\n");
		return (-1);
	}
	
	# open log information
	my ($status, $LOG) = read_file($g_log_file);
	if($status eq (-1))
	{
		print("can't open file $g_log_file\n");
		return $status;
	}
	
	# parse log information
	while(my $tmp_line = <$LOG>)
	{
		my $statistics_id = 0;
		while(1)
		{
			if(!(defined $g_statistics{$statistics_id}))
			{
				last;
			}
			my $formula_num = $g_statistics{$statistics_id}->{'formula'}->{'number'};
			for(my $j = 0; $j < $formula_num; $j++)
			{
				my $formula_valid = formula_valid_check($statistics_id,$j);
				if($formula_valid eq (-1))
				{
					print("formula_valid_check error $statistics_id $j\n");
					print("$tmp_line");
				}
				elsif($formula_valid eq 0)
				{
					next;
				}
				else
				{
					my $execute_valid = formula_execute($statistics_id,$j,$tmp_line);
					if($execute_valid eq (-1))
					{
						print("formula_valid_check error $statistics_id $j\n");
						print("$tmp_line");
					}
				}
			}
			$statistics_id++;
		}
	}
	# statistics result;
	my $statistics_id = 0;
	while(1)
	{
		if(!(defined $g_statistics{$statistics_id}))
		{
			last;
		}
		my @coordinate_x = ();
		my @coordinate_y = ();
		push @coordinate_x,0;
		push @coordinate_y,0;
		if(defined $g_statistics{$statistics_id}->{'var'}->{'coordinate_x'}
			and defined $g_statistics{$statistics_id}->{'var'}->{'coordinate_y'})
		{
			push @coordinate_x,$g_statistics{$statistics_id}->{'var'}->{'coordinate_x'}->{'value'};
			push @coordinate_y,$g_statistics{$statistics_id}->{'var'}->{'coordinate_y'}->{'value'};
		}
		elsif(defined $g_statistics{$statistics_id}->{'var'}->{'coordinate_xx'}
			and defined $g_statistics{$statistics_id}->{'var'}->{'coordinate_yy'})
		{
			my $number = $g_statistics{$statistics_id}->{'var'}->{'coordinate_xx'}->{'number'};
			for(my $j = 0; $j < $number; $j++)
			{
				push @coordinate_x,$g_statistics{$statistics_id}->{'var'}->{'coordinate_xx'}->{$j};
			}
			print "x number = $number\n";
			$number = $g_statistics{$statistics_id}->{'var'}->{'coordinate_yy'}->{'number'};
			for(my $j = 0; $j < $number; $j++)
			{
				push @coordinate_y,$g_statistics{$statistics_id}->{'var'}->{'coordinate_yy'}->{$j};
			}
			print "y number = $number\n";
		}
		else
		{
			print("result variable initial error: $statistics_id\n");
			return (-1);
		}

		my $x_num = @coordinate_x;
		my $y_num = @coordinate_y;
		my $graph_name = $g_statistics{$statistics_id}->{'graph_name'};
		my $x_name = $g_statistics{$statistics_id}->{'x_name'};
		my $y_name = $g_statistics{$statistics_id}->{'y_name'};
		print "x = @coordinate_x\n";
		print "y = @coordinate_y\n";
		if($x_num eq $y_num)
		{
			my $grap_return = line_grap($graph_name,\@coordinate_y,\@coordinate_x,$y_name,$x_name);
			if($grap_return eq 0)
			{
				print("line grap error: $statistics_id x = @coordinate_x; y = @coordinate_y\n");
				return(-1);
			}
		}
		else
		{
			print("result variable calculate error: $statistics_id xNum = $x_num, yNum = $y_num\n");
			return (-1);
		}
		$statistics_id++;
	}
	return 0;
	
}



sub read_file
{
	my ($file) = @_;
	@_ = ();

	open(my $FILE,$file) || return(-1,0);
	return(0,$FILE);
	
}



statistics();

