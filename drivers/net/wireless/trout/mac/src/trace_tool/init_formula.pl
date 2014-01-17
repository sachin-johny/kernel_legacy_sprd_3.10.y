#use strict;
#use warnings;

#my %g_statistics = ();



my $g_formula_file = 'statistics.cfg';
my $g_formula_debug = 0;
	
my %g_var_name;
my $g_graph_name_valid;
my $g_x_name_valid;
my $g_y_name_valid;
my @g_condition_start;
my @g_condition_type;
my $g_formula_idx;


sub global_init
{
	%g_var_name = ();
	$g_graph_name_valid = 0;
	$g_x_name_valid = 0;
	$g_y_name_valid = 0;
	@g_condition_start = ();
	@g_condition_type = ();
	$g_formula_idx = 0;

}



sub statistics_init
{
	open(my $FORMULA, $g_formula_file) || die "can't find file $g_formula_file\n\n";
	my @formula_team;
	my $formula_valid = 0;
	my $line_num = 0;
	my $statistics_id = 0;
	while(my $tmp_line=<$FORMULA>)
	{
		$line_num++;
		$tmp_line =~ s/\r?\n$//g;
		if($tmp_line =~ /^\s*$/)
		{
			next;
		}
		if($formula_valid eq 1)
		{
			if($tmp_line =~ /all start/)
			{
				print("formula start repeat at line:$line_num, content:$tmp_line\n");
				return (-1);
			}
			elsif($tmp_line =~ /all end/)
			{
				$formula_valid = 0;
				my $result = formula_init($statistics_id++,@formula_team);
				if($result eq -1)
				{
					print("formula initial error\n");
					return (-1);
				}
			}
			else
			{
				push @formula_team, $tmp_line;
			}
		}
		else	
		{
			if($tmp_line =~ /all start/)
			{
				@formula_team = ();
				$formula_valid = 1;
			}
			elsif($tmp_line =~ /all end/)
			{
				print("formula end repeat at line:$line_num, content:$tmp_line\n");
				return (-1);
			}
		}
	}
	close($FORMULA);
	return 0;

}



sub formula_init
{
	my ($statistics_id,@formula_team) = @_;
	@_ = ();

	my $return;
	global_init();
	# check formula
	for(my $i = 0; $i < @formula_team; $i++)
	{
		my $tmp_formula = $formula_team[$i];
		if($tmp_formula =~ /set graph_name\s*=\s*(.+)$/)
		{
			$g_graph_name_valid = 1;
			$g_statistics{$statistics_id}->{'graph_name'} = clear_str($1);
		}
		elsif($tmp_formula =~ /set x_name\s*=\s*(.+)$/)
		{
			$g_x_name_valid = 1;
			$g_statistics{$statistics_id}->{'x_name'} = clear_str($1);
		}
		elsif($tmp_formula =~ /set y_name\s*=\s*(.+)$/)
		{
			$g_y_name_valid = 1;
			$g_statistics{$statistics_id}->{'y_name'} = clear_str($1);
		}
		elsif($tmp_formula =~ /^\s*(.+)\s*=\s*get<(.+)>/)
		{
			$return = formula_init_get($statistics_id,$tmp_formula);
			if($return eq -1)
			{
				return (-1);
			}
			$g_formula_idx++;
		}
		elsif($tmp_formula =~ /when\s*\(/ or $tmp_formula =~ /after\s*\(/)
		{
			$return = formula_init_condition($statistics_id,$tmp_formula);
			if($return eq -1)
			{
				return (-1);
			}
			$g_formula_idx++;
		}
		elsif($tmp_formula =~ /when end/ or $tmp_formula =~ /after end/)
		{
			$return = formula_init_condition_end($statistics_id,$tmp_formula);
			if($return eq -1)
			{
				return (-1);
			}
		}
		elsif($tmp_formula =~ /\+/ or $tmp_formula =~ /-/ or $tmp_formula =~ /\*/ or $tmp_formula =~ /\//)
		{
			$return = formula_init_calculate($statistics_id,$tmp_formula);
			if($return eq -1)
			{
				return (-1);
			}
			$g_formula_idx++;
		}
		else
		{
			print("this formula isn't be defined at $statistics_id,$tmp_formula\n");
			return (-1);
		}
	}
	# check variable
	foreach my $tmp_var_name(keys(%g_var_name))
	{
		if($g_var_name{$tmp_var_name} ne 1)
		{
			print("this statistics formula have a variable isn't be defined at $statistics_id,$tmp_var_name\n");
			return (-1);
		}
	}
	my $tmp_num;
	$tmp_num = %g_var_name;
	if($tmp_num eq 0)
	{
		print("this statistics formula haven't any variable at $statistics_id\n");
		return (-1);
	}
	$g_statistics{$statistics_id}->{'var'}->{'number'} = $tmp_num;
	$return = formula_init_variable($statistics_id);
	if($return eq -1)
	{
		return (-1);
	}
	# check graph
	if($g_graph_name_valid ne 1 or $g_x_name_valid ne 1 or $g_y_name_valid ne 1)
	{
		print("this statistics graph info set error at $statistics_id\n");
		return (-1);
	}
	$tmp_num = @g_condition_start;
	if($tmp_num ne 0)
	{
		print("this statistics condition syntax start error at $statistics_id,id = @g_condition_start\n");
		return (-1);
	}
	$tmp_num = @g_condition_type;
	if($tmp_num ne 0)
	{
		print("this statistics condition type array opera error at $statistics_id,$tmp_num\n");
		return (-1);
	}
	return 0;

}


sub formula_init_get
{
	my ($statistics_id,$formula_line) = @_;
	@_ = ();

	my $result;
	my $var;
	my $tmp_line = clear_str($formula_line);
	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$g_formula_idx};
	$g_statistics{$statistics_id}->{'formula'}->{'number'}++;
	if($tmp_line =~ /^(.+)=get<(.+)>/)
	{
		$result = $1;
		$var = $2;
		#print "$var\n";
	}
	else
	{
		print("formula init get syntax error at $statistics_id, $formula_line\n");
		return (-1);
	}
	if($result !~ /[a-zA-Z]/)
	{
		print("formula init get result is a number at $statistics_id, $formula_line\n");
		return (-1);
	}
	else
	{
		$g_var_name{$result} = 1;
		$$formula_hash->{'action'} = 'get';
		$$formula_hash->{'result'} = $result;
		$$formula_hash->{'var1'} = $var;
		$$formula_hash->{'var2'} = 0;
		$$formula_hash->{'var3'} = 0;
		my $condition_num = @g_condition_start;
		$$formula_hash->{'run_check'}->{'number'} = $condition_num;
		for(my $i = 0; $i < $condition_num; $i++)
		{
			$$formula_hash->{'run_check'}->{$i}->{'valid'} = 0;
			$$formula_hash->{'run_check'}->{$i}->{'id'} = $g_condition_start[$i];
		}
	}

}


sub formula_init_condition
{
	my ($statistics_id,$formula_line) = @_;
	@_ = ();

	my $action1;
	my $action2;
	my $result;
	my $var1;
	my $var2;
	my $var3;
	my $tmp_line = clear_str($formula_line);
	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$g_formula_idx};
	$g_statistics{$statistics_id}->{'formula'}->{'number'}++;
	if($tmp_line =~ /when\(/)
	{
		$action1 = 'when';
	}
	elsif($tmp_line =~ /after\(/)
	{
		$action1 = 'after';
	}
	else
	{
		print("formula init condition syntax analyse error at $statistics_id, $formula_line, $tmp_line\n");
		return (-1);
	}
	$result = $g_formula_idx + 1;
	$var3 = 0;
	if($tmp_line =~ /define<(.+)>/)
	{
		$action2 = 'define';
		$var1 = $1;
		#print "$var1\n";
		$var2 = 0;
	}
	elsif($tmp_line =~ /\((.+)==(.+)\)/)
	{
		$action2 = 'eq';
		$var1 = $1;
		$var2 = $2;
	}
	elsif($tmp_line =~ /\((.+)!=(.+)\)/)
	{
		$action2 = 'ne';
		$var1 = $1;
		$var2 = $2;
	}
	elsif($tmp_line =~ /\((.+)>(.+)\)/)
	{
		$action2 = 'gt';
		$var1 = $1;
		$var2 = $2;
	}
	elsif($tmp_line =~ /\((.+)>=(.+)\)/)
	{
		$action2 = 'ge';
		$var1 = $1;
		$var2 = $2;
	}
	elsif($tmp_line =~ /\((.+)<(.+)\)/)
	{
		$action2 = 'lt';
		$var1 = $1;
		$var2 = $2;
	}
	elsif($tmp_line =~ /\((.+)<=(.+)\)/)
	{
		$action2 = 'le';
		$var1 = $1;
		$var2 = $2;
	}
	else
	{
		print("formula init condition syntax error at $statistics_id, $formula_line, $tmp_line\n");
		return (-1);
	}
	$$formula_hash->{'action'} = $action1.'_'.$action2;
	$$formula_hash->{'result'} = $result;
	$$formula_hash->{'var1'} = $var1;
	$$formula_hash->{'var2'} = $var2;
	$$formula_hash->{'var3'} = $var3;
	my $condition_num = @g_condition_start;
	$$formula_hash->{'run_check'}->{'number'} = $condition_num;
	for(my $i = 0; $i < $condition_num; $i++)
	{
		$$formula_hash->{'run_check'}->{$i}->{'valid'} = 0;
		$$formula_hash->{'run_check'}->{$i}->{'id'} = $g_condition_start[$i];
	}
	push @g_condition_start,$g_formula_idx;
	push @g_condition_type,$action1;
	return 0;

}


sub formula_init_condition_end
{
	my ($statistics_id,$formula_line) = @_;
	@_ = ();

	my $action1;
	my $action2;
	my $result;
	my $var1;
	my $var2;
	my $var3;
	my $tmp_line = clear_str($formula_line);
	if($tmp_line =~ /whenend/)
	{
		$action1 = 'when';
	}
	elsif($tmp_line =~ /afterend/)
	{
		$action1 = 'after';
	}
	else
	{
		print("formula init condition end syntax analyse error at $statistics_id, $formula_line\n");
		return (-1);
	}
	my $condition_id = pop(@g_condition_start);
	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$condition_id};
	my $start_line_id = $$formula_hash->{'result'};
	my $end_line_id = $g_formula_idx - 1;
	my $range_do_number = $end_line_id - $start_line_id + 1;
	$action2 = pop(@g_condition_type);
	if($action1 ne $action2)
	{
		print("formula init condition end syntax error at $statistics_id,$condition_id,$action2,$action1\n");
		return (-1);
	}

	if($start_line_id > $end_line_id)
	{
		print("formula init condition end process error at $statistics_id,$condition_id,$start_line_id,$end_line_id.\n");
		return (-1);
	}
	$$formula_hash->{'var3'} = $end_line_id;
	for(my $i = 0; $i < $range_do_number; $i++)
	{
		$$formula_hash->{'range_do'}->{$i} = 0;
	}
	return 0;

}


sub formula_init_calculate
{
	my ($statistics_id,$formula_line) = @_;
	@_ = ();

	my $action;
	my $result;
	my $var1;
	my $var2;
	my $tmp_line = clear_str($formula_line);
	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$g_formula_idx};
	$g_statistics{$statistics_id}->{'formula'}->{'number'}++;
	if($tmp_line =~ /^(.+)=(.+)\+(.+)/)
	{
		$action = 'add';
		$result = $1;
		$var1 = $2;
		$var2 = $3;
	}
	elsif($tmp_line =~ /^(.+)=(.+)-(.+)/)
	{
		$action = 'reduce';
		$result = $1;
		$var1 = $2;
		$var2 = $3;
	}
	elsif($tmp_line =~ /^(.+)=(.+)\*(.+)/)
	{
		$action = 'multiply';
		$result = $1;
		$var1 = $2;
		$var2 = $3;
	}
	elsif($tmp_line =~ /^(.+)=(.+)\/(.+)/)
	{
		$action = 'division';
		$result = $1;
		$var1 = $2;
		$var2 = $3;
	}
	else
	{
		print "formula init calculate syntax error at $statistics_id, $formula_line, $tmp_line\n";
		return (-1);
	}
	if($result !~ /[a-zA-Z]/)
	{
		print("formula init get result is a number at $statistics_id, $formula_line, $result\n");
		return (-1);
	}
	else
	{
		$g_var_name{$result} = 1;
		$$formula_hash->{'action'} = $action;
		$$formula_hash->{'result'} = $result;
		$$formula_hash->{'var1'} = $var1;
		$$formula_hash->{'var2'} = $var2;
		$$formula_hash->{'var3'} = 0;
		my $condition_num = @g_condition_start;
		$$formula_hash->{'run_check'}->{'number'} = $condition_num;
		for(my $i = 0; $i < $condition_num; $i++)
		{
			$$formula_hash->{'run_check'}->{$i}->{'valid'} = 0;
			$$formula_hash->{'run_check'}->{$i}->{'id'} = $g_condition_start[$i];
		}
		if($var1 =~ /[a-zA-Z]/)
		{
			if(!(defined $g_var_name{$var1}))
			{
				$g_var_name{$var1} = 0;
			}
		}
		if($var2 =~ /[a-zA-Z]/)
		{
			if(!(defined $g_var_name{$var2}))
			{
				$g_var_name{$var2} = 0;
			}
		}
	}

}


sub formula_init_variable
{
	my ($statistics_id) = @_;
	@_ = ();

	my $var_hash = \$g_statistics{$statistics_id}->{'var'};
	my $coordinate_single = 0;
	my $coordinate_mult = 0;
	foreach my $var_name(keys(%g_var_name))
	{
		if($var_name eq 'coordinate_xx' or $var_name eq 'coordinate_yy')
		{
			$coordinate_mult++;
			$$var_hash->{$var_name}->{'number'} = 0;
		}
		elsif($var_name eq 'coordinate_x' or $var_name eq 'coordinate_y')
		{
			$coordinate_single++;
			$$var_hash->{$var_name}->{'valid'} = 0;
			$$var_hash->{$var_name}->{'value'} = 0;
		}
		else
		{
			$$var_hash->{$var_name}->{'valid'} = 0;
			$$var_hash->{$var_name}->{'value'} = 0;
		}
	}
	if($coordinate_single ne 0 and $coordinate_mult ne 0)
	{
		print("formula init variable coordinate type error at $statistics_id,$coordinate_single,$coordinate_mult\n");
		return (-1);
	}
	elsif($coordinate_single + $coordinate_mult ne 2)
 	{
		print("formula init variable coordinate type error at $statistics_id,$coordinate_single,$coordinate_mult\n");
		return (-1);
	}
	return 0;
	
}


sub clear_str
{
	my ($str) = @_;
	@_ = ();

	my $replace;
	if($str =~ /<(.*)>/)
	{
		$replace = $1;
	}
	$str = clear_formula($str);
	$str =~ s/,//g;
	$str =~ s/<.*>/<$replace>/g;
	return $str;

}


sub clear_formula
{
	my ($str) = @_;
	@_ = ();

	$str =~ s/\s+//g;
	$str =~ s/"//g;
	$str =~ s/;//g;
	return $str;

}


1;
