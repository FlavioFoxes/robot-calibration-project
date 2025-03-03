# Directory
IDIR = include
CC = g++
CFLAGS = -I$(IDIR)

ODIR = obj
LDIR = lib

SRCDIR = src

# File di dipendenze
_DEPS = utils.h tricycle.h leastSquares.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

# File oggetto
_OBJ = utils.o leastSquares.o tricycle.o main.o
OBJ = $(patsubst %, $(ODIR)/% ,$(_OBJ))

# Regola predefinita "all" che crea la cartella obj e costruisce il programma
all: $(ODIR) calibration.exe

# Creazione della cartella obj
$(ODIR):
	mkdir -p $(ODIR)

# Regola per compilare i file oggetto
$(ODIR)/%.o: $(SRCDIR)/%.cpp $(DEPS) | $(ODIR)
	$(CC) -c -o $@ $< $(CFLAGS)

# Creazione dell'eseguibile
calibration.exe: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

# Pulizia
.PHONY: clean
clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~
