import { createTheme } from "@mui/material/styles";

const drcTheme = createTheme({
  components: {
    MuiTableCell: {
      styleOverrides: {
        root: {
          borderBottom: 0,
        },
      },
    },
  },
});

export default drcTheme;
