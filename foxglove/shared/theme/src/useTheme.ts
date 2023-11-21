import { useMediaQuery } from "@mui/material";
import { createTheme } from "@mui/material/styles";
import type { Theme } from "@mui/material/styles";

const useTheme = (): Theme => {
  const prefersDarkMode = useMediaQuery("(prefers-color-scheme: dark)");

  const theme = createTheme({
    palette: {
      mode: prefersDarkMode ? "dark" : "light",
    },
    components: {
      MuiTableRow: {
        styleOverrides: {
          root: {
            "&:last-child td, &:last-child th": {
              border: 0,
            },
          },
        },
      },
    },
  });

  return theme;
};

export default useTheme;
