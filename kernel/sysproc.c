#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "vm.h"
#include "file.h"
#include "sleeplock.h"
#include "fs.h"

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
        argaddr(0, &addr);
        argint(1, &length);
        argint(2, &prot);
        argint(3, &flags);
        argint(4, &fd);
        argint(5, &offset);

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
                        if (prot & PROT_READ) perm |= PTE_R;
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

uint64
sys_munmap(void)
{
  uint64 addr;
  struct mmap_area *ma = 0;
  struct proc *p = myproc();
  
  // 사용자로부터 addr 인자(0번째)를 가져옴
  argaddr(0, &addr);
  
  // mmap_area 배열에서 해당 영역을 찾음
  acquire(&mmap_lock);
  for (ma = mmap_areas; ma < &mmap_areas[MAX_MMAP_AREAS]; ma++) {
    // 소유자가 현재 프로세스이고, mmap() 호출 시 사용했던 addr이 일치하는지 확인
    if (ma->p == p && ma->addr == addr) {
      break; // 찾음
    }
  }
  
  if (ma == 0 || ma >= &mmap_areas[MAX_MMAP_AREAS]) {
    release(&mmap_lock);
    return -1; // 실패
  }
  
  uint64 va_start = MMAPBASE + ma->addr;
  int npages = ma->length / PGSIZE;
  struct file *f = ma->f;

  ma->p = 0;
  ma->f = 0;
  ma->length = 0;
  ma->addr = 0;
  ma->offset = 0;
  ma->prot = 0;
  ma->flags = 0;

  release(&mmap_lock);

  if (npages > 0) {
    uvmunmap(p->pagetable, va_start, npages, 1);
  }

  if (f) {
    fileclose(f);
  }

  return 1;
}

uint64
sys_freemem(void)
{
  return freemem();
}

int
handle_page_fault(uint64 fault_va, uint64 scause)
{
  struct proc *p = myproc();
  struct mmap_area *ma = 0;
  
  // mmap_area 검색 및 검사
  acquire(&mmap_lock);
  
  // 현재 프로세스에 속하고, 폴트 주소를 포함하는 mmap_area 찾음
  for (ma = mmap_areas; ma < &mmap_areas[MAX_MMAP_AREAS]; ma++) {
    if (ma->p == p && (MMAPBASE + ma->addr) <= fault_va && fault_va < (MMAPBASE + ma->addr + ma->length)) {
                        break; // 찾음
    }
  }

  // 매핑 영역을 찾지 못한 경우 (잘못된 접근)
  if (ma == 0 || ma >= &mmap_areas[MAX_MMAP_AREAS]) {
    release(&mmap_lock);
    return -1; // 실패
  }

  // 쓰기 금지 위반 검사 (쓰기 폴트인데, PROT_WRITE 권한이 없는 경우)
  if (scause == 15 && !(ma->prot & PROT_WRITE)) {
    release(&mmap_lock);
    return -1; // 실패
  }

  // 페이지 할당 및 매핑 (폴트가 발생한 1페이지만)

  // 폴트가 발생한 가상 주소의 시작점 (페이지 정렬)
  uint64 va_page_start = PGROUNDDOWN(fault_va);

  // 필요한 정보 복사
  int flags = ma->flags;
  int prot = ma->prot;
  int offset = ma->offset + (va_page_start - (MMAPBASE + ma->addr));

  struct file *f = ma->f;
  if (f) filedup(f); // 파일 참조 카운트 증가

  release(&mmap_lock);                                            
  
  // 물리 페이지 할당 및 채우기
  char *mem = kalloc();
  if (mem == 0) {
    if (f) fileclose(f);
    return -1;
  }

  if (flags & MAP_ANONYMOUS) {
    memset(mem, 0, PGSIZE); // 익명 매핑은 0으로 채움
  }
  else {
    ilock(f->ip);
    readi(f->ip, 0, (uint64)mem, offset, PGSIZE);
    iunlock(f->ip);
    fileclose(f);
  }

  // 페이지 테이블 매핑
  int perm = PTE_U;
  if (prot & PROT_READ) perm |= PTE_R;
  if (prot & PROT_WRITE) perm |= PTE_W;
  
  if (mappages(p->pagetable, va_page_start, PGSIZE, (uint64)mem, perm) != 0) {
    kfree(mem);
    return -1;
  }

  return 1; // 성공
}
