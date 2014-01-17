sub get_c_code_file
{
#########################################
#		变量定义
#########################################
	my ($dir_path) = @_;
	my $temp;
	my $dir;
	my $c_code_size;
	my @c_code_file;
	my @next_dir_str;
#########################################
#		判断文件夹
#########################################
	if(!(-e $dir_path))
	{
		print("$dir_path:no such file or dir\n");
		return ();		
	}
	if(!opendir($dir,$dir_path))
	{
		print("error:Dir\"$dir_path\" can not open\n");
		return ();
	}
#########################################
#		大循环
#########################################
	while($temp=readdir($dir))
	{
		if($temp =~ /.+\.((c)|(h)|(cpp))$/)
		{
			$c_code_size = @c_code_file;
			@c_code_file[$c_code_size] = "$dir_path\/$temp";
		}
		elsif($temp =~ /^\.$/ || $temp =~ /^\.\.$/)
		{	
			next;
		}
		elsif(-d "$dir_path/$temp")
		{
			#print("$dir_path\/$temp\n");
			@next_dir_str = get_c_code_file("$dir_path\/$temp");
			if(@next_dir_str != ())
			{
				@c_code_file = (@c_code_file,@next_dir_str);
			}
		}
	}
	close($dir_path);
	return @c_code_file;
}

1;

