# record all the cpu usage

# while true; do (echo "%CPU %MEM ARGS $(date)" && ps -e -o pcpu,pmem,args --sort=pcpu | cut -d" " -f1-5 | tail) >> ps.log; sleep 5; done

FILE=cpu_usage-$(date +"%Y_%m_%d_%I_%M_%p").log
while true; do (top -bn1 | grep '%Cpu' | tail -1 | grep -P '(....|...) id,'|awk '{print 100-$8}' >> $FILE); sleep 1; done

# top batch mode (run until killed), grep, tail, grep pattern ( here | means 'or'), $8 means the 8th word in the line, and the reason why we subutract it from 100 is id stands for idle time. (CPU usage + idle time = 100) 
