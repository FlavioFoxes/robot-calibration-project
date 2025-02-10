IDIR =include
CC=g++
CFLAGS=-I$(IDIR)

ODIR=obj
LDIR =lib

_DEPS = utils.h tricycle.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = utils.o tricycle.o main.o 
OBJ = $(patsubst %, $(ODIR)/% ,$(_OBJ))

SRCDIR = src

$(ODIR)/%.o: $(SRCDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

prova.exe: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 