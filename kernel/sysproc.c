#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "vm.h"

uint64
sys_exit(void)
{
  int n;
  argint(0, &n);
  kexit(n);
  return 0;  // not reached
}

uint64
sys_getpid(void)
{
  return myproc()->pid;
}

uint64
sys_fork(void)
{
  return kfork();
}

uint64
sys_wait(void)
{
  uint64 p;
  argaddr(0, &p);
  return kwait(p);
}

uint64
sys_sbrk(void)
{
  uint64 addr;
  int t;
  int n;

  argint(0, &n);
  argint(1, &t);
  addr = myproc()->sz;

  if(t == SBRK_EAGER || n < 0) {
    if(growproc(n) < 0) {
      return -1;
    }
  } else {
    // Lazily allocate memory for this process: increase its memory
    // size but don't allocate memory. If the processes uses the
    // memory, vmfault() will allocate it.
    if(addr + n < addr)
      return -1;
    myproc()->sz += n;
  }
  return addr;
}

uint64
sys_pause(void)
{
  int n;
  uint ticks0;

  argint(0, &n);
  if(n < 0)
    n = 0;
  acquire(&tickslock);
  ticks0 = ticks;
  while(ticks - ticks0 < n){
    if(killed(myproc())){
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock);
  }
  release(&tickslock);
  return 0;
}

uint64
sys_kill(void)
{
  int pid;

  argint(0, &pid);
  return kkill(pid);
}

// return how many clock tick interrupts have occurred
// since start.
uint64
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

uint64
sys_getpname(void)
{
        int pid;

        argint(0, &pid);
        return getpname(pid);
}

uint64
sys_getnice(void)
{
        int pid;

        argint(0, &pid);
        return getnice(pid);
}

uint64
sys_setnice(void)
{
        int pid, value;

        argint(0, &pid);
        argint(1, &value);
        return setnice(pid, value);
}

uint64
sys_ps(void)
{
        int pid;

        argint(0, &pid);
        ps(pid);

        return 0; // ps 시스템콜은 리턴값 없음
}

uint64
sys_meminfo(void)
{
        return meminfo();
}

uint64
sys_waitpid(void)
{
        int pid;

        argint(0, &pid);
        return waitpid(pid);
}

uint64
sys_mmap(void)
{
        uint64 addr;
        int length, prot, flags, fd, offset;

        // 사용자 공간에서 인수를 가져옴
        if (argaddr(0, &addr) < 0 || argint(1, &length) < 0 || argint(2, &prot) < 0 ||
                        argint(3, &flags) < 0 || argint(4, &fd) < 0 || argint(5, &offset) < 0) {
                return 0; // 실패 시 0 반환
        }

        // addr과 length가 페이지 정렬되었는지 확인
        if ((addr % PGSIZE) != 0 || (length % PGSIZE) != 0) {
                return 0; // 페이지 정렬되지 않으면 실패
        }

        // 현재 프로세스 정보 가져오기
        struct proc *p = myproc();
        struct file *f = 0;

        // fd와 MAP_ANONYMOUS 플래그 일치 여부 확인
        if (flags & MAP_ANONYMOUS) {
                if (fd != -1) return 0; // 익명 매핑인데 fd가 -1이 아니면 실패
        }
        else {
                if (fd < 0 || fd >= NOFILE || (f = p->ofile[fd]) == 0) {
                        return 0; // 파일 매핑인데 유효하지 않은 fd이면 실패
                }

                // prot와 파일 열기 권한 일치 여부 확인
                if ((prot & PROT_READ) && !f->readable) {
                        return 0; // 읽기 권한이 없는데 PROT_READ 요청 시 실패
                }
                if ((prot & PROT_WRITE) && !f->writable) {
                        return 0; // 쓰기 권한이 없는데 PROT_WRITE 요청 시 실패
                }
        }

        // mmap_area 배열에서 빈 슬롯을 찾아 기록
        acquire(&mmap_lock);
        struct mmap_area *ma = 0;
        for (int i = 0; i < MAX_MMAP_AREAS; i++) {
                if (mmap_areas[i].p == 0) { // p가 0이면 비어있는 슬롯
                        ma = &mmap_areas[i];
                        break;
                }
        }

        if (ma == 0) { // 빈 슬롯이 없으면 실패
                release(&mmap_lock);
                return 0;
        }

        // 매핑 정보 기록
        ma->p = p;
        ma->addr = addr;
        ma->length = length;
        ma->f = (flags & MAP_ANONYMOUS) ? 0 : filedup(f);
        ma->prot = prot;
        ma->flags = flags;
        ma->offset = offset;

        release(&mmap_lock);

        // MAP_POPULATE 처리
        if (flags & MAP_POPULATE) {
                // MMAPBASE + addr부터 length만큼 반복
                for (uint64 current_va = MMAPBASE + addr; current_va < MMAPBASE + addr + length; current_va += PGSIZE) {
                        // 물리 페이지 할당
                        char *mem = kalloc();
                        if (mem == 0) {
                                // 메모리 할당 실패 시, 이미 할당된 자원을 해제해야 함(munmap 로직 필요)
                                // munmap(addr); munmap 구현 후 호출
                                return 0;
                        }

                        // 페이지 채우기
                        if (flags & MAP_ANONYMOUS) {
                                memset(mem, 0, PGSIZE); // 익명이면 0으로 채움
                        }
                        else {
                                // 파일 매핑이면 파일 데이터를 읽어와 채움
                                ilock(f->ip);
                                readi(f->ip, 0, (uint64)mem, ma->offset + (current_va - (MMAPBASE + addr)), PGSIZE);
                                iunlock(f->ip);
                        }

                        // 페이지 테이블 매핑 (PTE_U: 사용자 접근 가능)
                        int perm = PTE_U;
                        if (prot & PROT_READ) prem |= PTE_R;
                        if (prot & PROT_WRITE) perm |= PTE_W;

                        if (mappages(p->pagetable, current_va, PGSIZE, (uint64)mem, perm) != 0) {
                                kfree(mem);
                                // munmap(addr); munmap 구현 후 호출
                                return 0;
                        }
                }
        }

        // 성공 시 시작 주소 반환
        return MMAPBASE + addr;
}
