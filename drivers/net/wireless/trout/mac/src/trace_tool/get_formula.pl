
#my %g_statistics = ();
my $file_name = 'statistics.cfg';
#my $file_name = 'data.txt';

	
my $g_debug_mode = 1;




sub statistics_init
{
	my ($handle) = 0;
	my ($line_content) = '';
	my $dest_num = -1;
	my ($formula_num) = 0;
	my $action = '';
	my ($result, $var1, $var2, $var3) = 0;
	my (@var_arr) = ();
	my ($var_num) = 0;
	my ($tmp) = 0;
	my ($condition_num) = 0;
	my (@condition_line) = ();
	

	open($handle, $file_name);
	
	while ($line_content = <$handle>)
	{
		chomp($line_content);
		$line_content =~ s/^\s+//;
		$line_content =~ s/^\t+//;
		if ($g_debug_mode == 1)
		{
			print("--line content:  $line_content\n");
		}
		if ($line_content =~ /all +start/)
		{
			$dest_num++;
			$formula_num = 0;
			$condition_num = 0;
			$condition_line = ();
			$g_statistics{$dest_num}->{'var'}->{'coordinate_xx'}->{'number'} = 0;
			$g_statistics{$dest_num}->{'var'}->{'coordinate_yy'}->{'number'} = 0;
		}
		if ($dest_num lt 0)
		{
			next;
		}
		elsif ($line_content =~ /set\s+graph_name\s*=\s*/)
		{
			$g_statistics{$dest_num}=$';
			if ($g_statistics{$dest_num} =~ /;/)
			{	
				$g_statistics{$dest_num}=$`;
			}
			if ($g_debug_mode == 1)
			{
				print("grap name is: $g_statistics{$dest_num}\n");
			}
		}
		elsif ($line_content =~ /set\s+x_name\s*=\s*/)
		{
			$g_statistics{$dest_num}->{"x_name"}=$';
			if ($2 =~ /;/)
			{
				$g_statistics{$dest_num}->{"x_name"}=$`;
			}
			if ($g_debug_mode == 1)
			{
				print("x_name is:");
				print($g_statistics{$dest_num}->{'x_name'});
				print("\n");
			}
		}
		elsif ($line_content =~ /set\s+y_name\s*=\s*/)
		{
			$g_statistics{$dest_num}->{"y_name"}=$';
			if ($2 =~ /;/)
			{
				$g_statistics{$dest_num}->{"y_name"}=$`;
			}
			if ($g_debug_mode == 1)
			{
				print("y_name is:");
				print($g_statistics{$dest_num}->{'x_name'});
				print("\n");
			}
		}
		#$formula_num = 0;
		#R = get(V);
		elsif ($line_content =~ /(\w+)(\s*=\s*)(get)(<\w+>)/)
		{
			print("get\n");
			$result = $1;
			$action = $3;
			$var1 = $4;
			if ($var1 =~ /\w+/)
			{
				$var1 = $&;
			}
			@var_arr = ($result, $var1);
			$var_num = 2;
			&make_formula(\@var_arr, $var_num, \@condition_line, 
					$condition_num, $action, $dest_num, $formula_num);
			$formula_num++;
		}
		#R = A+B;
		#R = A-B;
		#R = A*B;
		#R = A/B;
		elsif ($line_content =~ /(\w+)(\s*=\s*)(\w+)(\s*[\+\-\*\/]\s*)(\w+)/)
		{
			$action = $4;
			@var_arr = ($1, $3, $5);
			$var_num = 3;
			&make_formula(\@var_arr, $var_num, \@condition_line,
					 $condition_num, $action, $dest_num, $formula_num);
			$formula_num++;
		}
		#R = average(V, N)
		#R = sum(V, N)
		elsif ($line_content =~ /(\w+)(\s*=\s*)(average|sum)(\(.*\))/)
		{
			$result = $1;
			$action = $3;

			if ($4 =~ /(\w+)(\s*,\s*)(\w+)/)
			{
				$var1 = $1;
				$var2 = $3;
			}
			@var_arr = ($result, $var1, $var2);
			$var_num = 3;
			&make_formula(\@var_arr, $var_num, \@condition_line, 
					$condition_num, $action, $dest_num, $formula_num);
			$formula_num++;
		}
		#when after
		elsif ($line_content =~ /(when|after)(\s*\(.*\))/)
		{
			$tmp = $2;
			$action=$1;

			if ($tmp =~ /define/)
			{
				$tmp = $';
				if ($tmp =~ /".*"/)
				{
					$tmp = $&;
					$tmp =~ s/\"//g;
					@var_arr = ($formula_num+2, $tmp);
					$action=$action.' define';
					$var_num = 2;
				}
			}
			else
			{
				if ($tmp =~ /(\w+)(\W+)(\w+)/)
				{
					@var_arr = ($formula_num+2, $1, $3);
					$var_num = 3;
					$action = $action.$2;
				}
			}

			&make_formula(\@var_arr, $var_num, \@condition_line, 
					$condition_num, $action, $dest_num, $formula_num);
			
			$condition_line[$condition_num] = $formula_num;
			$formula_num++;
			$condition_num++;
		}
		#when end | after end
		elsif ($line_content =~ /(when|after)(end)/)
		{
			$p_tmp = $g_statistics{$dest_num}->{'formula'}->{$formula_num};
			$tmp = $formula_num - $condition_line[$condition_num-1];
			for ($i = 0; $i < $tmp; $i++)
			{
				($$p_tmp)->{'range_do'}->{$i} = 0;	
			}
			$tmp->{$condition_line[$condition_num-1]-1}->{'var3'}=$formula;			
			$condition_num--;
		}
	}
	$g_statistics{'number'} = $dest_num;
	if ($g_debug_num)
	{
		print("dest num: ");
		print($g_statistics{'number'});
		print("\n");
	}
	return 0;
}


sub make_action
{
	my ($action) = @_;

	if ($action =~ /\+/)
	{
		$action = 'add';
	}
	elsif ($action =~ /\-/)
	{
		$action = 'reduce';
	}
	elsif ($action =~ /\*/)
	{
		$action = 'multiply';
	}
	elsif ($action =~ /\//)
	{
		$action = 'division';
	}
	elsif ($action =~ /when\s+define/)
	{
		$action = 'when_define';
	}
	elsif ($action =~ /when\s*==/)
	{
		$action = 'when_eq';
	}
	elsif ($action =~ /when\s*!=/)
	{
		$action = 'when_ne';
	}
	elsif ($action =~ /when\s*>=/)
	{
		$action = 'when_ge';
	}
	elsif ($action =~ /when\s*<=/)
	{
		$action = 'when_le';
	}
	elsif ($action =~ /when\s*>/)
	{
		$action = 'when_gt';
	}
	elsif ($action =~ /when\s*</)
	{
		$action = 'when_lt';
	}
	elsif ($action =~ /after\s+define/)
	{
		$action = 'after_define';
	}
	elsif ($action =~ /after\s*==/)
	{
		$action = 'after_eq';
	}
	elsif ($action =~ /after\s*!=/)
	{
		$action = 'after_ne';
	}
	elsif ($action =~ /after\s*>=/)
	{
		$action = 'after_ge';
	}
	elsif ($action =~ /after\s*<=/)
	{
		$action = 'after_le';
	}
	elsif ($action =~ /after\s*>/)
	{
		$action = 'after_gt';
	}
	elsif ($action =~ /after\s*</)
	{
		$action = 'after_lt';
	}

	return $action;
}
sub make_formula
{
	my ($vars, $var_count, $condition_line, $condition_num, $action, $dest_num, $formula_num) = @_;
	@_ = ();
	my ($i) = 0;
	my ($p_tmp) = 0;
	my ($result) = $$vars[0];
	
	$action = &make_action($action);
	if ($g_debug_mode == 1)
	{
		print("resut: $result\n");
		print("action: $action\n");
	}

	unless ($action =~ /get|define/)
	{
		print("hhh\n");
		for ($i = 1; $i < $var_count; $i++ )
		{
			if ($$vars[$i] =~ /[a-zA-Z]+/)
			{
				if ($$vars[$i] =~ /coordinate_xx|coordinate_yy/)
				{
					
					$g_statistics{$dest_num}->{'var'}->{$&}->{'number'}++;
					if ($g_debug_mode == 1)
					{
						print("{var}->{coordinate_xx}: ");
						print($g_statistics{$dest_num}->{'var'}->{$&}->{'number'}++);
						print("\n");
					}
					
				}
				else
				{
					$g_statistics{$dest_num}->{'var'}->{$$vars[$i]}->{'valid'} = 0;
					$g_statistics{$dest_num}->{'var'}->{$$vars[$i]}->{'value'} = 0;
					if ($g_debug_mode == 1)
					{
						print("VAR$i, $$vars[$i]\n");
					}
				}
			}
		}
	}

	$g_statistics{$dest_num}->{'formula'}->{'number'} = $formula_num+1;
	$p_tmp = $g_statistics{$dest_num}->{'formula'}->{$formula_num};
	($$p)->{'action'} = $action;
	($$p)->{'result'} = $result;

	if ($var_count ge 1)
	{
		($$p_tmp)->{'var1'} = $$vars[1];
	}

	if ($var_count ge 2)
	{
		($$p_tmp)->{'var2'} = $$vars[2];
	}
	if ($var_count ge 3)
	{
		($$p_tmp)->{'var3'} = $$vars[3];
	}
	if ($g_debug_mode == 1)
	{
		print("op1: $$vars[1]\n");
		print("op2: $$vars[2]\n");
		print("op3: $$vars[3]\n");
	}

	($$p_tmp)->{'run_check'}->{'number'} = $condition_num;

	for ($i = 0; $i lt $condition_num; $i++)
	{
		($$p_tmp)->{'rum_check'}->{$i}->{'valid'} = 0;	
		($$p_tmp)->{'rum_check'}->{$i}->{'id'} = $$condition_line[$i];	
		if ($g_debug_mode == 1)
		{
			print("condition line: $$condition_line[$i]\n");
		}
	}

	if ($g_debug_mode == 1)
	{
		print("******************************************************************\n");
	}

}


1;
