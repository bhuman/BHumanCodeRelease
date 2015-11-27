/**
* @file bhumanemu.cpp
* Implementation of a dummy libbhuman replication.
* @author Colin Graf
*/

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/resource.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "bhuman.h"

#define ALLOWED_FRAMEDROPS 3

int fd = -1;
LBHData* data = (LBHData*)MAP_FAILED;
sem_t* sem = SEM_FAILED;
int frameDrops = ALLOWED_FRAMEDROPS + 1;

void close()
{
  fprintf(stderr, "libbhuman: Stop.\n");

  // unmap the shared memory
  if(data != MAP_FAILED)
  {
    munmap(data, sizeof(LBHData));
    data = (LBHData*)MAP_FAILED;
  }

  // close shared memory
  if(fd != -1)
  {
    close(fd);
    fd = -1;
  }

  // close semaphore
  if(sem != SEM_FAILED)
  {
    sem_close(sem);
    sem = SEM_FAILED;
  }

  fprintf(stderr, "libbhuman: Stopped.\n");
}

int create()
{
  fprintf(stderr, "libbhuman: Start.\n");

  // created shared memory
  if((fd = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR)) == -1)
  {
    perror("libbhuman: shm_open");
    close();
    return -1;
  }

  if(ftruncate(fd, sizeof(LBHData)) == -1)
  {
    perror("libbhuman: ftruncate");
    close();
    return -1;
  }

  // map the shared memory
  if((data = (LBHData*)mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED)
  {
    perror("libbhuman: mmap");
    close();
    return -1;
  }
  memset(data, 0, sizeof(LBHData));

  // open semaphore
  if((sem = sem_open(LBH_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0)) == SEM_FAILED)
  {
    perror("libbhuman: sem_open");
    close();
    return -1;
  }

  strcpy(data->robotName, "Nao");
  return 0;
}

int main(int argc, char* argv[])
{
  if(create() != 0)
    return EXIT_FAILURE;

  while(usleep(20 * 1000) == 0)
  {
    int sval;
    if(sem_getvalue(sem, &sval) == 0)
    {
      if(sval < 1)
      {
        sem_post(sem);
        frameDrops = 0;
      }
      else
      {
        if(frameDrops == 0)
          fprintf(stderr, "libbhuman: dropped frame.\n");
        frameDrops++;
      }
    }
  }

  close();
  return EXIT_SUCCESS;
}
