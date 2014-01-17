require "make_excel.pl";
use strict;
my ($g_file_name) = "cpu.txt";
my ($g_out_file_name) = "cpu_cost.xls";
my ($g_line);
my ($g_print_en) = 1;

my (@g_irq_cpu);
my (@g_rx_workqueue_cpu);
my (@g_event_cpu);
my (@g_thread1_cpu);

my ($g_irq_turn) = 1;
my ($g_rx_turn)  = 0;
my ($g_event_turn) = 0;
my ($g_thread1_turn) = 0;
sub put_into_excel
{
	my ($irq,$rx,$event,$thread1) = @_;
	my @array;
	my $sum;
	$sum = $irq + $rx + $event + $thread1;
	@array[0] = '';
	@array[1] = $irq;
	@array[2] = $rx;
	@array[3] = $event;
	@array[4] = $thread1;
	@array[5] = '';
	@array[6] = $sum;
	get_content(\@array);
}

sub build_excel_format
{
	my @array;
	@array[0] = 'TX';
	@array[1] = 'irq/41-wlan0(%)';
	@array[2] = 'rx_workqueue(%)';
	@array[3] = 'event_wq/0(%)';
	@array[4] = 'trace_thread1(%)';
	@array[5] = '';
	@array[6] = 'cpu_sum(%)';
	get_content(\@array);
}

sub build_excel_average
{
	my ($a_irq,$a_rx,$a_event,$a_thread1) = @_;
	my @array;
	my $a_sum = $a_irq + $a_rx + $a_event + $a_thread1;
	@array[0] = 'AverageVal(%)';
	@array[1] = $a_irq;
	@array[2] = $a_rx;
	@array[3] = $a_event;
	@array[4] = $a_thread1;
	@array[5] = '';
	@array[6] = $a_sum;
	get_content(\@array);
}
sub cpu_statistics
{
	my ($string) = @_;
	my $size = 0;

	my $to_excel_irq;
	my $to_excel_rx;
	my $to_excel_event;
	my $to_excel_thread1;
	if($g_print_en == 1)
	{
#		printf($string."abc\n");
	}
	if($string =~ /irq\/(\d+)-wlan0/)
	{
		if($string =~ /\d+\s+(\d+)\%/)
		{
			#printf(" irq,the cpu rate is:".$1."\n");
			$size = @g_irq_cpu;
			@g_irq_cpu[$size] = $1;
			$g_irq_turn = 1;
		}
	}
	elsif($string =~ /rx\_workqueue/)
	{
		if($string =~ /\d+\s+(\d+)\%/)
		{
			#printf("rx,the cpu rate is:".$1."\n");
			$size = @g_rx_workqueue_cpu;
			@g_rx_workqueue_cpu[$size] = $1;
			$g_rx_turn = 1;
		}
	}
	elsif($string =~ /event\_wq/)
	{
		if($string =~ /\d+\s+(\d+)\%/)
		{
			#printf("event,the cpu rate is:".$1."\n");
			$size = @g_event_cpu;
			@g_event_cpu[$size] = $1;
			$g_event_turn = 1;
		}
	}
	elsif($string =~ /trace\_thread1/)
	{
		if($string =~ /\d+\s+(\d+)\%/)
		{
			#printf("thread,the cpu rate is:".$1."\n");
			$size = @g_thread1_cpu;
			@g_thread1_cpu[$size] = $1;
			$g_thread1_turn = 1;
		}
	}
	if($g_irq_turn == 1 && $g_rx_turn == 1 && $g_event_turn == 1 && $g_thread1_turn == 1)
	{
		$g_irq_turn = 1;
		$g_rx_turn  = 0;
		$g_event_turn = 0;
		$g_thread1_turn = 0;
		
		$size = @g_irq_cpu;
		$to_excel_irq = $g_irq_cpu[$size - 1];
	
		$size = @g_rx_workqueue_cpu;
		$to_excel_rx = $g_rx_workqueue_cpu[$size - 1];

		$size = @g_event_cpu;
		$to_excel_event = $g_event_cpu[$size - 1];
		
		$size = @g_thread1_cpu;
		$to_excel_thread1 = $g_thread1_cpu[$size - 1];
		
		put_into_excel($to_excel_irq,$to_excel_rx,$to_excel_event,$to_excel_thread1);
	}
}
sub make_average
{
	my $average_irq = 0;
	my $average_rx = 0;
	my $average_event = 0;
	my $average_thread1 = 0;
	my $size;
	my $i;
	$size = @g_irq_cpu;
	#printf("irq_cpu size is:".$size);
	for($i = 0;$i < $size;$i++)
	{
		$average_irq += $g_irq_cpu[$i];
	}
	if($size != 0)
	{
		$average_irq /= $size;	
 	}

 	
	$size = @g_rx_workqueue_cpu;
	for($i = 0;$i < $size;$i++)
	{
		$average_rx += $g_rx_workqueue_cpu[$i];
	}
	if($size != 0)
	{
		$average_rx /= $size;
	}

	
	$size = @g_event_cpu;
	for($i = 0;$i < $size;$i++)
	{
		$average_event += $g_event_cpu[$i];
	}
	if($size != 0)
	{
		$average_event /= $size;
	}

	
	$size = @g_thread1_cpu;
	for($i = 0;$i < $size;$i++)
	{
		$average_thread1 += $g_thread1_cpu[$i];
	}
	if($size != 0)
	{
		$average_thread1 /= $size;
	}
	build_excel_average($average_irq,$average_rx,$average_event,$average_thread1);

}
build_excel_format();
open(file,$g_file_name) ||  die("can not open the file\n");
while($g_line = <file>)
{
	cpu_statistics($g_line);
}
make_average();
create_new_file($g_out_file_name);
